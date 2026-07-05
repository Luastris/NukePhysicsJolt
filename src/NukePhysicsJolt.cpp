// NukePhysicsJolt — the physics service provider (roadmap 1.1).
//
// An ordinary plugin (unified model): exports "plugin", provides()="physics",
// queryService() returns the iPhysics implementation. The ENGINE owns the fixed-step
// loop and the Collider/Rigidbody components (World::UpdatePhysics); this module only
// simulates behind the POD seam (service/iPhysics.h). Links NukeEngine.dll (import
// lib) — plugins EXTEND the engine, so engine API like nuke::Jobs is used directly.
// Jolt v5.5.0 is vendored statically (deps/JoltPhysics).

// Jolt first (its headers configure themselves via compile definitions from the target).
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemWithBarrier.h>
#include <Jolt/Core/FixedSizeFreeList.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLockInterface.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/ContactListener.h>

// Engine headers last (they do `using namespace std;` internally).
#include <interface/NUKEEInteface.h>   // NUKEModule (unified plugin model)
#include <service/iPhysics.h>          // the contract this module provides
#include <API/Model/Jobs.h>            // engine worker pool (roadmap 2.4)

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

using std::cout;
using std::endl;
using namespace nuke;

// ---- Jolt collision layers: static world vs everything that moves -------------------
namespace Layers
{
	static constexpr JPH::ObjectLayer NON_MOVING = 0;
	static constexpr JPH::ObjectLayer MOVING     = 1;
	static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
}
namespace BPLayers
{
	static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
	static constexpr JPH::BroadPhaseLayer MOVING(1);
	static constexpr JPH::uint NUM_LAYERS = 2;
}

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
{
public:
	JPH::uint GetNumBroadPhaseLayers() const override { return BPLayers::NUM_LAYERS; }
	JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override
	{
		return layer == Layers::NON_MOVING ? BPLayers::NON_MOVING : BPLayers::MOVING;
	}
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override
	{
		return layer == BPLayers::NON_MOVING ? "NON_MOVING" : "MOVING";
	}
#endif
};

class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
	bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer bpLayer) const override
	{
		// Statics never collide with statics; everything else collides.
		return layer != Layers::NON_MOVING || bpLayer != BPLayers::NON_MOVING;
	}
};

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter
{
public:
	bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override
	{
		return a != Layers::NON_MOVING || b != Layers::NON_MOVING;
	}
};

// Buffers contact transitions during PhysicsSystem::Update. Jolt calls these from its
// WORKER threads mid-step, so the buffer is mutex-guarded; the engine drains it after
// the step (fetchContacts) on the fixed-update thread.
class ContactCollector final : public JPH::ContactListener
{
public:
	void OnContactAdded(const JPH::Body& a, const JPH::Body& b,
	                    const JPH::ContactManifold& manifold, JPH::ContactSettings&) override
	{
		NukeContactEvent e;
		e.bodyA = a.GetID().GetIndexAndSequenceNumber();
		e.bodyB = b.GetID().GetIndexAndSequenceNumber();
		e.phase = 0;
		JPH::RVec3 p = manifold.GetWorldSpaceContactPointOn1(0);
		e.point[0] = (float)p.GetX(); e.point[1] = (float)p.GetY(); e.point[2] = (float)p.GetZ();
		e.normal[0] = manifold.mWorldSpaceNormal.GetX();
		e.normal[1] = manifold.mWorldSpaceNormal.GetY();
		e.normal[2] = manifold.mWorldSpaceNormal.GetZ();
		std::lock_guard<std::mutex> lock(m_mutex);
		m_events.push_back(e);
	}

	void OnContactRemoved(const JPH::SubShapeIDPair& pair) override
	{
		NukeContactEvent e;
		e.bodyA = pair.GetBody1ID().GetIndexAndSequenceNumber();
		e.bodyB = pair.GetBody2ID().GetIndexAndSequenceNumber();
		e.phase = 1;
		std::lock_guard<std::mutex> lock(m_mutex);
		m_events.push_back(e);
	}

	int Drain(NukeContactEvent* out, int max)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		const int n = (int)std::min<size_t>(m_events.size(), (size_t)max);
		std::copy(m_events.begin(), m_events.begin() + n, out);
		m_events.erase(m_events.begin(), m_events.begin() + n);
		return n;
	}

private:
	std::mutex m_mutex;
	std::vector<NukeContactEvent> m_events;
};

// Primitive query/body shape factory (Box/Sphere/Capsule — shared by createBody and the
// shape-cast/overlap queries).
static JPH::RefConst<JPH::Shape> MakePrimitiveShape(int shape, const float halfExtents[3],
                                                    float radius, float halfHeight)
{
	switch (shape)
	{
		case 1: return new JPH::SphereShape(std::max(1e-4f, radius));
		case 2: return new JPH::CapsuleShape(std::max(1e-4f, halfHeight), std::max(1e-4f, radius));
		default:
			return new JPH::BoxShape(JPH::Vec3(std::max(1e-4f, halfExtents[0]),
			                                   std::max(1e-4f, halfExtents[1]),
			                                   std::max(1e-4f, halfExtents[2])));
	}
}

// Jolt job system backed by the ENGINE's worker pool (roadmap 2.4): solver jobs are
// scheduled straight onto nuke::Jobs instead of a private thread pool, so ALL parallel
// work shares the same per-core-pinned workers. Barriers come from JobSystemWithBarrier
// (WaitForJobs executes remaining barrier jobs on the CALLING thread, so a saturated
// pool can never deadlock the fixed step).
class NukeJobSystem final : public JPH::JobSystemWithBarrier
{
public:
	NukeJobSystem(JPH::uint maxJobs, JPH::uint maxBarriers)
	{
		nuke::Jobs::Init();   // no-op when the host already inited (both hosts do at boot)
		JobSystemWithBarrier::Init(maxBarriers);
		m_jobs.Init(maxJobs, maxJobs);
	}

	int GetMaxConcurrency() const override
	{
		return nuke::Jobs::WorkerCount() + 1;   // + the calling (fixed) thread: it executes barrier jobs too
	}

	JobHandle CreateJob(const char* name, JPH::ColorArg color,
	                    const JobFunction& fn, JPH::uint32 numDependencies = 0) override
	{
		// Same free-list scheme as Jolt's own JobSystemThreadPool.
		JPH::uint32 index;
		for (;;)
		{
			index = m_jobs.ConstructObject(name, color, this, fn, numDependencies);
			if (index != AvailableJobs::cInvalidObjectIndex)
				break;
			JPH_ASSERT(false, "No jobs available!");
			std::this_thread::sleep_for(std::chrono::microseconds(100));
		}
		Job* job = &m_jobs.Get(index);
		JobHandle handle(job);           // keep a reference — the job may complete immediately
		if (numDependencies == 0)
			QueueJob(job);
		return handle;
	}

protected:
	void QueueJob(Job* job) override
	{
		job->AddRef();                   // released after Execute
		nuke::Jobs::Schedule([job] { job->Execute(); job->Release(); });
	}
	void QueueJobs(Job** jobs, JPH::uint count) override
	{
		for (JPH::uint i = 0; i < count; ++i)
			QueueJob(jobs[i]);
	}
	void FreeJob(Job* job) override { m_jobs.DestructObject(job); }

private:
	using AvailableJobs = JPH::FixedSizeFreeList<Job>;
	AvailableJobs m_jobs;
};

// ---- iPhysics implementation ----------------------------------------------------------
class JoltPhysics final : public iPhysics
{
public:
	bool init() override
	{
		if (m_system) return true;   // idempotent

		// Route Jolt's diagnostics into the engine log — without these a Debug assert
		// hits JPH_BREAKPOINT and kills the process SILENTLY.
		JPH::Trace = [](const char* fmt, ...)
		{
			char buf[1024];
			va_list args; va_start(args, fmt);
			vsnprintf(buf, sizeof(buf), fmt, args);
			va_end(args);
			cout << "[Jolt]\t\t" << buf << endl;
		};
		JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = [](const char* expr, const char* msg,
		                                             const char* file, JPH::uint line) -> bool
		{
			cout << "[Jolt]\t\tASSERT " << (file ? file : "?") << ":" << line << ": " << expr
			     << (msg ? " - " : "") << (msg ? msg : "") << endl;
			return false;   // log, don't breakpoint
		};)

		JPH::RegisterDefaultAllocator();
		if (!JPH::Factory::sInstance)
		{
			JPH::Factory::sInstance = new JPH::Factory();
			JPH::RegisterTypes();
		}
		// Update()'s scratch scales with kMaxBodies (16k bodies wanted ~14 MB) — 32 MB has headroom.
		m_tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(32 * 1024 * 1024);
		// Solver jobs run on the ENGINE's shared per-core-pinned pool (nuke::Jobs, 2.4).
		m_jobSystem = std::make_unique<NukeJobSystem>(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers);

		m_system = std::make_unique<JPH::PhysicsSystem>();
		m_system->Init(kMaxBodies, 0, kMaxBodyPairs, kMaxContacts,
		               m_bpInterface, m_objVsBpFilter, m_pairFilter);
		m_system->SetContactListener(&m_contacts);
		cout << "[NukePhysicsJolt]\tJolt " << JPH_VERSION_MAJOR << "." << JPH_VERSION_MINOR
		     << "." << JPH_VERSION_PATCH << " ready (engine pool, " << nuke::Jobs::WorkerCount() << " workers)" << endl;
		return true;
	}

	void reset() override
	{
		if (!m_system) return;
		JPH::BodyInterface& bi = m_system->GetBodyInterface();
		for (JPH::uint32 raw : m_bodies)
		{
			JPH::BodyID id(raw);
			bi.RemoveBody(id);
			bi.DestroyBody(id);
		}
		m_bodies.clear();
	}

	void setGravity(const float g[3]) override
	{
		if (m_system) m_system->SetGravity(JPH::Vec3(g[0], g[1], g[2]));
	}

	uint64_t createBody(const NukeBodyDesc& d) override
	{
		if (!m_system) return 0;

		int motionCode = d.motion;
		JPH::RefConst<JPH::Shape> shape;
		switch (d.shape)
		{
			case 1: case 2:
				shape = MakePrimitiveShape(d.shape, d.halfExtents, d.radius, d.halfHeight); break;
			case 3:   // triangle-soup mesh: convex hull (dynamic-capable) or static MeshShape
			{
				if (!d.meshVerts || d.meshVertCount < 3)
				{
					cout << "[NukePhysicsJolt]\tmesh collider without mesh data - skipped" << endl;
					return 0;
				}
				if (d.convex)
				{
					JPH::Array<JPH::Vec3> points;
					points.reserve(d.meshVertCount);
					for (int i = 0; i < d.meshVertCount; ++i)
						points.push_back(JPH::Vec3(d.meshVerts[i * 3 + 0],
						                           d.meshVerts[i * 3 + 1],
						                           d.meshVerts[i * 3 + 2]));
					JPH::ConvexHullShapeSettings hs(points);
					JPH::Shape::ShapeResult r = hs.Create();
					if (r.HasError())
					{
						cout << "[NukePhysicsJolt]\tconvex hull failed: " << r.GetError() << endl;
						return 0;
					}
					shape = r.Get();
				}
				else
				{
					JPH::TriangleList tris;
					const int triCount = d.meshVertCount / 3;
					tris.reserve(triCount);
					for (int t = 0; t < triCount; ++t)
					{
						const float* v = d.meshVerts + t * 9;
						tris.push_back(JPH::Triangle(JPH::Float3(v[0], v[1], v[2]),
						                             JPH::Float3(v[3], v[4], v[5]),
						                             JPH::Float3(v[6], v[7], v[8])));
					}
					JPH::MeshShapeSettings ms(tris);
					JPH::Shape::ShapeResult r = ms.Create();
					if (r.HasError())
					{
						cout << "[NukePhysicsJolt]\tmesh shape failed: " << r.GetError() << endl;
						return 0;
					}
					shape = r.Get();
					if (motionCode == 1)   // Jolt mesh shapes cannot be SIMULATED (kinematic is fine - mass provided below)
					{
						cout << "[NukePhysicsJolt]\tnon-convex mesh collider cannot be DYNAMIC - forced STATIC (set convex=true for a hull)" << endl;
						motionCode = 0;
					}
				}
				break;
			}
			default:
				shape = MakePrimitiveShape(0, d.halfExtents, d.radius, d.halfHeight);
				break;
		}

		const JPH::EMotionType motion = motionCode == 1 ? JPH::EMotionType::Dynamic
		                              : motionCode == 2 ? JPH::EMotionType::Kinematic
		                                                : JPH::EMotionType::Static;
		const JPH::ObjectLayer layer = motionCode == 0 ? Layers::NON_MOVING : Layers::MOVING;

		JPH::BodyCreationSettings bcs(shape,
			JPH::RVec3(d.pos[0], d.pos[1], d.pos[2]),
			JPH::Quat(d.quat[0], d.quat[1], d.quat[2], d.quat[3]).Normalized(),
			motion, layer);
		bcs.mFriction       = d.friction;
		bcs.mRestitution    = d.restitution;
		bcs.mLinearDamping  = d.linearDamping;
		bcs.mAngularDamping = d.angularDamping;
		bcs.mGravityFactor  = d.useGravity ? 1.0f : 0.0f;
		bcs.mIsSensor       = d.isTrigger;   // sensor: contact events, no collision response
		if (motion == JPH::EMotionType::Kinematic && d.shape == 3 && !d.convex)
		{
			// A triangle-soup MeshShape cannot derive its own mass - creating the motion
			// properties would assert and then crash (null MotionProperties). Kinematic
			// bodies don't integrate forces, so a solid box of the mesh bounds is exact
			// enough; it only matters if something reads the mass.
			const JPH::AABox bounds = shape->GetLocalBounds();
			const JPH::Vec3 size = JPH::Vec3::sMax(bounds.GetSize(), JPH::Vec3::sReplicate(0.01f));
			bcs.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
			bcs.mMassPropertiesOverride.SetMassAndInertiaOfSolidBox(size, 1000.0f);
			bcs.mMassPropertiesOverride.ScaleToMass(d.mass > 0.0f ? d.mass : 1000.0f);
		}
		else if (motion == JPH::EMotionType::Dynamic && d.mass > 0.0f)
		{
			bcs.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
			bcs.mMassPropertiesOverride.mMass = d.mass;
		}

		JPH::BodyInterface& bi = m_system->GetBodyInterface();
		JPH::Body* body = bi.CreateBody(bcs);
		if (!body) { cout << "[NukePhysicsJolt]\tcreateBody FAILED (body pool full?)" << endl; return 0; }
		bi.AddBody(body->GetID(), JPH::EActivation::Activate);
		m_bodies.insert(body->GetID().GetIndexAndSequenceNumber());
		return body->GetID().GetIndexAndSequenceNumber();
	}

	void destroyBody(uint64_t handle) override
	{
		if (!m_system || !handle) return;
		JPH::BodyID id((JPH::uint32)handle);
		if (!m_bodies.erase(id.GetIndexAndSequenceNumber())) return;   // unknown/stale
		JPH::BodyInterface& bi = m_system->GetBodyInterface();
		bi.RemoveBody(id);
		bi.DestroyBody(id);
	}

	void setBodyPose(uint64_t handle, const float pos[3], const float quat[4]) override
	{
		if (!m_system || !handle) return;
		m_system->GetBodyInterface().SetPositionAndRotation(
			JPH::BodyID((JPH::uint32)handle),
			JPH::RVec3(pos[0], pos[1], pos[2]),
			JPH::Quat(quat[0], quat[1], quat[2], quat[3]).Normalized(),
			JPH::EActivation::Activate);
	}

	void moveKinematic(uint64_t handle, const float pos[3], const float quat[4], float dt) override
	{
		if (!m_system || !handle) return;
		JPH::BodyID id((JPH::uint32)handle);
		if (dt <= 0.0f)
		{
			setBodyPose(handle, pos, quat);
			return;
		}
		// A body that is NOT kinematic has no motion properties to derive velocities
		// from - Body::MoveKinematic would dereference null and kill the process.
		// Whatever the mismatch (a backend-forced static, a stale handle), degrade to
		// a pose teleport instead of crashing.
		if (m_system->GetBodyInterface().GetMotionType(id) != JPH::EMotionType::Kinematic)
		{
			setBodyPose(handle, pos, quat);
			return;
		}
		// MoveKinematic derives linear/angular velocity for the step, so contacting
		// dynamic bodies are CARRIED instead of left behind (moving platforms).
		m_system->GetBodyInterface().ActivateBody(id);
		m_system->GetBodyInterface().MoveKinematic(id,
			JPH::RVec3(pos[0], pos[1], pos[2]),
			JPH::Quat(quat[0], quat[1], quat[2], quat[3]).Normalized(), dt);
	}

	bool getBodyPose(uint64_t handle, float pos[3], float quat[4]) override
	{
		if (!m_system || !handle) return false;
		JPH::BodyID id((JPH::uint32)handle);
		if (!m_bodies.count(id.GetIndexAndSequenceNumber())) return false;
		JPH::RVec3 p; JPH::Quat q;
		m_system->GetBodyInterface().GetPositionAndRotation(id, p, q);
		pos[0] = (float)p.GetX(); pos[1] = (float)p.GetY(); pos[2] = (float)p.GetZ();
		quat[0] = q.GetX(); quat[1] = q.GetY(); quat[2] = q.GetZ(); quat[3] = q.GetW();
		return true;
	}

	void setLinearVelocity(uint64_t handle, const float v[3]) override
	{
		if (!m_system || !handle) return;
		m_system->GetBodyInterface().SetLinearVelocity(
			JPH::BodyID((JPH::uint32)handle), JPH::Vec3(v[0], v[1], v[2]));
	}

	void getLinearVelocity(uint64_t handle, float v[3]) override
	{
		v[0] = v[1] = v[2] = 0.0f;
		if (!m_system || !handle) return;
		JPH::Vec3 vel = m_system->GetBodyInterface().GetLinearVelocity(JPH::BodyID((JPH::uint32)handle));
		v[0] = vel.GetX(); v[1] = vel.GetY(); v[2] = vel.GetZ();
	}

	void setAngularVelocity(uint64_t handle, const float v[3]) override
	{
		if (!m_system || !handle) return;
		m_system->GetBodyInterface().SetAngularVelocity(
			JPH::BodyID((JPH::uint32)handle), JPH::Vec3(v[0], v[1], v[2]));
	}

	void getAngularVelocity(uint64_t handle, float v[3]) override
	{
		v[0] = v[1] = v[2] = 0.0f;
		if (!m_system || !handle) return;
		JPH::Vec3 av = m_system->GetBodyInterface().GetAngularVelocity(JPH::BodyID((JPH::uint32)handle));
		v[0] = av.GetX(); v[1] = av.GetY(); v[2] = av.GetZ();
	}

	int fetchContacts(NukeContactEvent* out, int max) override
	{
		if (!out || max <= 0) return 0;
		return m_contacts.Drain(out, max);
	}

	void addForce(uint64_t handle, const float f[3]) override
	{
		if (!m_system || !handle) return;
		m_system->GetBodyInterface().AddForce(JPH::BodyID((JPH::uint32)handle), JPH::Vec3(f[0], f[1], f[2]));
	}

	void addImpulse(uint64_t handle, const float i[3]) override
	{
		if (!m_system || !handle) return;
		m_system->GetBodyInterface().AddImpulse(JPH::BodyID((JPH::uint32)handle), JPH::Vec3(i[0], i[1], i[2]));
	}

	void step(float dt) override
	{
		if (!m_system) return;
		m_system->Update(dt, 1, m_tempAllocator.get(), m_jobSystem.get());
	}

	bool raycast(const float from[3], const float dir[3], float maxDist,
	             uint64_t& hitBody, float hitPoint[3], float hitNormal[3]) override
	{
		if (!m_system) return false;
		JPH::Vec3 d(dir[0], dir[1], dir[2]);
		if (d.LengthSq() < 1e-12f || maxDist <= 0.0f) return false;
		d = d.Normalized() * maxDist;
		JPH::RRayCast ray{ JPH::RVec3(from[0], from[1], from[2]), d };
		JPH::RayCastResult hit;
		if (!m_system->GetNarrowPhaseQuery().CastRay(ray, hit)) return false;

		hitBody = hit.mBodyID.GetIndexAndSequenceNumber();
		JPH::RVec3 p = ray.GetPointOnRay(hit.mFraction);
		hitPoint[0] = (float)p.GetX(); hitPoint[1] = (float)p.GetY(); hitPoint[2] = (float)p.GetZ();
		hitNormal[0] = hitNormal[1] = 0.0f; hitNormal[2] = 0.0f;
		{
			JPH::BodyLockRead lock(m_system->GetBodyLockInterface(), hit.mBodyID);
			if (lock.Succeeded())
			{
				JPH::Vec3 n = lock.GetBody().GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, p);
				hitNormal[0] = n.GetX(); hitNormal[1] = n.GetY(); hitNormal[2] = n.GetZ();
			}
		}
		return true;
	}

	bool shapeCast(const NukeShapeDesc& s, const float from[3], const float quat[4],
	               const float dir[3], float maxDist,
	               uint64_t& hitBody, float hitPoint[3], float hitNormal[3]) override
	{
		if (!m_system) return false;
		JPH::Vec3 d(dir[0], dir[1], dir[2]);
		if (d.LengthSq() < 1e-12f || maxDist <= 0.0f) return false;
		d = d.Normalized() * maxDist;

		JPH::RefConst<JPH::Shape> shape = MakePrimitiveShape(s.shape, s.halfExtents, s.radius, s.halfHeight);
		JPH::RMat44 start = JPH::RMat44::sRotationTranslation(
			JPH::Quat(quat[0], quat[1], quat[2], quat[3]).Normalized(),
			JPH::RVec3(from[0], from[1], from[2]));
		JPH::RShapeCast cast(shape, JPH::Vec3::sReplicate(1.0f), start, d);
		JPH::ShapeCastSettings settings;
		JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
		m_system->GetNarrowPhaseQuery().CastShape(cast, settings, JPH::RVec3::sZero(), collector);
		if (!collector.HadHit()) return false;

		const JPH::ShapeCastResult& hit = collector.mHit;
		hitBody = hit.mBodyID2.GetIndexAndSequenceNumber();
		hitPoint[0] = (float)hit.mContactPointOn2.GetX();
		hitPoint[1] = (float)hit.mContactPointOn2.GetY();
		hitPoint[2] = (float)hit.mContactPointOn2.GetZ();
		// mPenetrationAxis points from the cast shape INTO the hit body; the surface normal
		// faces back at the caster.
		JPH::Vec3 n = hit.mPenetrationAxis;
		if (n.LengthSq() > 1e-12f) n = -n.Normalized(); else n = JPH::Vec3::sZero();
		hitNormal[0] = n.GetX(); hitNormal[1] = n.GetY(); hitNormal[2] = n.GetZ();
		return true;
	}

	int overlap(const NukeShapeDesc& s, const float pos[3], const float quat[4],
	            uint64_t* outBodies, int max) override
	{
		if (!m_system || !outBodies || max <= 0) return 0;
		JPH::RefConst<JPH::Shape> shape = MakePrimitiveShape(s.shape, s.halfExtents, s.radius, s.halfHeight);
		JPH::RMat44 xform = JPH::RMat44::sRotationTranslation(
			JPH::Quat(quat[0], quat[1], quat[2], quat[3]).Normalized(),
			JPH::RVec3(pos[0], pos[1], pos[2]));
		JPH::CollideShapeSettings settings;
		JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
		m_system->GetNarrowPhaseQuery().CollideShape(shape, JPH::Vec3::sReplicate(1.0f), xform,
		                                             settings, JPH::RVec3::sZero(), collector);
		int n = 0;
		for (const JPH::CollideShapeResult& hit : collector.mHits)
		{
			const uint64_t id = hit.mBodyID2.GetIndexAndSequenceNumber();
			bool dup = false;                     // a body can report several sub-shape hits
			for (int i = 0; i < n; ++i) if (outBodies[i] == id) { dup = true; break; }
			if (dup) continue;
			outBodies[n++] = id;
			if (n >= max) break;
		}
		return n;
	}

	void shutdown()
	{
		reset();
		m_system.reset();
		m_jobSystem.reset();
		m_tempAllocator.reset();
		if (JPH::Factory::sInstance)
		{
			JPH::UnregisterTypes();
			delete JPH::Factory::sInstance;
			JPH::Factory::sInstance = nullptr;
		}
	}

private:
	static constexpr JPH::uint kMaxBodies    = 16384;
	static constexpr JPH::uint kMaxBodyPairs = 32768;
	static constexpr JPH::uint kMaxContacts  = 16384;

	BPLayerInterfaceImpl              m_bpInterface;
	ObjectVsBroadPhaseLayerFilterImpl m_objVsBpFilter;
	ObjectLayerPairFilterImpl         m_pairFilter;
	ContactCollector                  m_contacts;

	std::unique_ptr<JPH::TempAllocatorImpl>    m_tempAllocator;
	std::unique_ptr<NukeJobSystem>             m_jobSystem;   // solver jobs on nuke::Jobs (2.4)
	std::unique_ptr<JPH::PhysicsSystem>        m_system;
	std::unordered_set<JPH::uint32>            m_bodies;      // live handles (reset/validation)
};
static JoltPhysics gPhysics;

// ---- Plugin export (unified plugin model) ---------------------------------------------
struct NukePhysicsJoltModule : public NUKEModule
{
	NukePhysicsJoltModule()
	{
		strcpy(title, "Jolt Physics");
		strcpy(description, "Rigid-body physics backed by Jolt (jrouwe/JoltPhysics v5.5.0, vendored static).");
		strcpy(author, "Luastris");
		strcpy(site, "https://luastris.com");
		strcpy(version, "0.1.0.0");
		tags = { "physics", "jolt", "rigidbody" };
	}

	const char* provides() override { return "physics"; }
	void*       queryService() override { return static_cast<iPhysics*>(&gPhysics); }

	void OnLoad() override {}          // Collider/Rigidbody are ENGINE components — already registered
	void Run(AppInstance* inst) override { instance = inst; stopped = false; }   // driven by World's fixed step
	bool HasSettings() override { return false; }
	void Settings() override {}
	void Shutdown() override
	{
		gPhysics.shutdown();           // loader revoked the "physics" service before calling this
		stopped = true;
	}
};

extern "C" BOOST_SYMBOL_EXPORT NukePhysicsJoltModule plugin;
NukePhysicsJoltModule plugin;
