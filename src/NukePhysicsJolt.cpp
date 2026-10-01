// NukePhysicsJolt — the "physics" service provider (iPhysics behind the POD seam
// service/iPhysics.h). The engine owns the fixed-step loop and the Collider/Rigidbody
// components; this module only simulates. Jolt v5.5.0 is vendored statically.

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
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>

// Engine headers last (they do `using namespace std;` internally).
#include <interface/NUKEEInteface.h>   // NUKEModule (unified plugin model)
#include <service/iPhysics.h>          // the contract this module provides
#include <API/Model/Jobs.h>            // engine worker pool

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <memory>
#include <mutex>
#include <thread>
#include <map>
#include <unordered_set>
#include <cmath>
#include <unordered_map>
#include <vector>

using std::cout;
using std::endl;

// Jolt trace hook: a REAL variadic function — MSVC converts a (const char*, ...) lambda to
// this pointer type, clang does not.
static void JoltTrace(const char* fmt, ...)
{
	char buf[1024];
	va_list args; va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	cout << "[Jolt]\t\t" << buf << endl;
}
using namespace nuke;

// ---- Jolt collision layers: static world vs everything that moves ----
namespace ObjLayers   // Jolt OBJECT layers; nuke::Layers are the engine's RENDER layers
{
	static constexpr JPH::ObjectLayer NON_MOVING = 0;
	static constexpr JPH::ObjectLayer MOVING     = 1;
	static constexpr JPH::ObjectLayer SOFT       = 2;   // world-space soft bodies (flags, tablecloths)
	static constexpr JPH::ObjectLayer SOFT_PROXY = 3;   // collides ONLY with soft cloth (body proxies)
	static constexpr JPH::ObjectLayer SOFT_LOCAL = 4;   // anchor-space cloth: proxies ONLY, never the world
	static constexpr JPH::ObjectLayer NUM_LAYERS = 5;
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
		return layer == ObjLayers::NON_MOVING ? BPLayers::NON_MOVING : BPLayers::MOVING;
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
		// Cloth-only layers live in the MOVING broadphase and want nothing from NON_MOVING.
		if (layer == ObjLayers::SOFT_PROXY || layer == ObjLayers::SOFT_LOCAL)
			return bpLayer == BPLayers::MOVING;
		return layer != ObjLayers::NON_MOVING || bpLayer != BPLayers::NON_MOVING;
	}
};

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter
{
public:
	bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override
	{
		// Anchor-space cloth touches ONLY the body proxies (it sits near the origin, the
		// world there is unrelated); proxies touch ONLY cloth.
		if (a == ObjLayers::SOFT_LOCAL || b == ObjLayers::SOFT_LOCAL)
			return (a == ObjLayers::SOFT_LOCAL ? b : a) == ObjLayers::SOFT_PROXY;
		if (a == ObjLayers::SOFT_PROXY || b == ObjLayers::SOFT_PROXY)
			return (a == ObjLayers::SOFT_PROXY ? b : a) == ObjLayers::SOFT;
		return a != ObjLayers::NON_MOVING || b != ObjLayers::NON_MOVING;
	}
};

// Buffers contact transitions during PhysicsSystem::Update. Jolt calls these from its WORKER
// threads mid-step, so the buffer is mutex-guarded; the engine drains it after the step.
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
		// the closing speed before the solve (surface velocities at the contact point, spin included)
		{
			const JPH::Vec3 va = a.IsStatic() ? JPH::Vec3::sZero() : a.GetPointVelocity(p);
			const JPH::Vec3 vb = b.IsStatic() ? JPH::Vec3::sZero() : b.GetPointVelocity(p);
			e.speed = std::fabs((va - vb).Dot(manifold.mWorldSpaceNormal));
		}
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

// Box/Sphere/Capsule factory shared by createBody and the shape-cast/overlap queries.
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

// Jolt job system backed by the engine's nuke::Jobs pool, so all parallel work shares one set
// of workers. Barriers must come from JobSystemWithBarrier: WaitForJobs runs remaining barrier
// jobs on the CALLING thread, which is what keeps a saturated pool from deadlocking the step.
class NukeJobSystem final : public JPH::JobSystemWithBarrier
{
public:
	NukeJobSystem(JPH::uint maxJobs, JPH::uint maxBarriers)
	{
		nuke::Jobs::Init();   // no-op when the host already inited
		JobSystemWithBarrier::Init(maxBarriers);
		m_jobs.Init(maxJobs, maxJobs);
	}

	// A pool worker's `job->Release()` runs AFTER the barrier lets step() return — destroying
	// this system right then frees the job list under a live release (teardown assert at
	// best, use-after-free at worst; scene destroys and Quit both hit it). Wait them out.
	~NukeJobSystem() override
	{
		while (m_inFlight.load(std::memory_order_acquire) > 0)
			std::this_thread::yield();
	}

	int GetMaxConcurrency() const override
	{
		return nuke::Jobs::WorkerCount() + 1;   // + the calling thread: it runs barrier jobs too
	}

	JobHandle CreateJob(const char* name, JPH::ColorArg color,
	                    const JobFunction& fn, JPH::uint32 numDependencies = 0) override
	{
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
		JobHandle handle(job);           // reference FIRST: the job may complete immediately
		if (numDependencies == 0)
			QueueJob(job);
		return handle;
	}

protected:
	void QueueJob(Job* job) override
	{
		job->AddRef();                   // released after Execute
		m_inFlight.fetch_add(1, std::memory_order_relaxed);
		nuke::Jobs::Schedule([job, this]
		{
			job->Execute();
			job->Release();
			m_inFlight.fetch_sub(1, std::memory_order_release);
		});
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
	std::atomic<int> m_inFlight{ 0 };
};

// ---- iPhysics implementation ----
class JoltPhysics final : public iPhysics
{
public:
	bool init() override
	{
		if (m_system) return true;   // idempotent

		// These hooks are REQUIRED: without them a Debug assert hits JPH_BREAKPOINT and kills
		// the process silently.
		JPH::Trace = &JoltTrace;   // real function: clang cannot convert a lambda to a variadic fn ptr
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
		// Update()'s scratch scales with kMaxBodies: 16k bodies want ~14 MB.
		m_tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(32 * 1024 * 1024);
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
		// Characters MUST go first: their inner bodies die with them, before the body sweep.
		for (auto& kv : m_characters)
			m_charVsChar.Remove(kv.second.character);
		m_characters.clear();
		// Constraints must leave the system before their bodies (stale body refs assert in step).
		for (auto& kv : m_vehicles)
		{
			m_system->RemoveStepListener(kv.second);
			m_system->RemoveConstraint(kv.second);
		}
		m_vehicles.clear();
		m_vehBase.clear();
		if (m_hoverStepper.added) { m_system->RemoveStepListener(&m_hoverStepper); m_hoverStepper.added = false; }
		m_hovers.clear();
		m_timeScale.clear();
		for (auto& kv : m_joints) m_system->RemoveConstraint(kv.second);
		m_joints.clear();
		for (auto& kv : m_constraints) m_system->RemoveConstraint(kv.second.first);
		m_constraints.clear();
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
					if (motionCode == 1)   // Jolt mesh shapes cannot be simulated (kinematic is fine)
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

		// Center-of-mass shift (vehicles/boats): wrap the shape, geometry stays put.
		if (d.comOffset[0] != 0.0f || d.comOffset[1] != 0.0f || d.comOffset[2] != 0.0f)
		{
			JPH::OffsetCenterOfMassShapeSettings os(
				JPH::Vec3(d.comOffset[0], d.comOffset[1], d.comOffset[2]), shape);
			JPH::Shape::ShapeResult r = os.Create();
			if (!r.HasError()) shape = r.Get();
		}

		const JPH::EMotionType motion = motionCode == 1 ? JPH::EMotionType::Dynamic
		                              : motionCode == 2 ? JPH::EMotionType::Kinematic
		                                                : JPH::EMotionType::Static;
		const JPH::ObjectLayer layer = d.softOnly    ? ObjLayers::SOFT_PROXY
		                             : motionCode == 0 ? ObjLayers::NON_MOVING : ObjLayers::MOVING;

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
		if (motion != JPH::EMotionType::Static && d.shape == 3)
		{
			// Mesh-based moving bodies can't derive mass (a MeshShape has none, a degenerate
			// hull has zero volume) and Jolt asserts — override with a solid box of the bounds.
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
		// A non-kinematic body has no motion properties: Body::MoveKinematic would deref null.
		if (m_system->GetBodyInterface().GetMotionType(id) != JPH::EMotionType::Kinematic)
		{
			setBodyPose(handle, pos, quat);
			return;
		}
		// MoveKinematic derives step velocities, so contacting dynamic bodies get carried.
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

	// Soft bodies carry their motion per vertex: the rigid velocity is only a summary.
	static JPH::SoftBodyMotionProperties* SoftMP(JPH::Body& b)
	{
		return b.IsSoftBody() ? static_cast<JPH::SoftBodyMotionProperties*>(b.GetMotionProperties()) : nullptr;
	}
	// A vehicle on a scaled chassis: forces act over the real dt, so torque/brake go x s^2 and the
	// suspension frequency x s (damping is a ratio) — the car then obeys its own, slower clock.
	void VehicleTimeScale(JPH::uint32 chassis, float s)
	{
		for (auto& kv : m_vehicles)
		{
			JPH::VehicleConstraint* vc = kv.second;
			if (!vc || vc->GetVehicleBody()->GetID().GetIndexAndSequenceNumber() != chassis) continue;
			auto bit = m_vehBase.find(kv.first);
			if (bit == m_vehBase.end()) continue;
			const VehicleBase& vb = bit->second;
			if (vb.type == 1)
			{
				auto* tc = static_cast<JPH::TrackedVehicleController*>(vc->GetController());
				tc->GetEngine().mMaxTorque = vb.maxTorque * s * s;
				for (int t = 0; t < (int)JPH::ETrackSide::Num; ++t) tc->GetTracks()[t].mMaxBrakeTorque = vb.trackBrake * s * s;
				for (const VehicleBase::WheelBase& w : vb.wheels) w.ws->mSuspensionSpring.mFrequency = w.frequency * s;
				continue;
			}
			static_cast<JPH::WheeledVehicleController*>(vc->GetController())->GetEngine().mMaxTorque = vb.maxTorque * s * s;
			for (const VehicleBase::WheelBase& w : vb.wheels)
			{
				auto* ws = static_cast<JPH::WheelSettingsWV*>(w.ws);
				ws->mSuspensionSpring.mFrequency = w.frequency * s;
				ws->mMaxBrakeTorque = w.brake * s * s;
				ws->mMaxHandBrakeTorque = w.handBrake * s * s;
			}
		}
		for (auto& kv : m_hovers)
			if (kv.second.body == chassis) kv.second.s = s;   // forces x s^2, lift spring x s (StepHovers)
	}
	void setBodyTimeScale(uint64_t handle, float s) override
	{
		if (!m_system || !handle) return;
		const JPH::uint32 id = (JPH::uint32)handle;
		if (!m_bodies.count(id)) return;
		if (s < 0.0f) s = 0.0f;
		JPH::BodyLockWrite lock(m_system->GetBodyLockInterface(), JPH::BodyID(id));
		if (!lock.Succeeded()) return;
		JPH::Body& b = lock.GetBody();
		if (b.IsStatic()) return;
		JPH::SoftBodyMotionProperties* soft = SoftMP(b);
		if (std::fabs(s - 1.0f) < 1e-6f)
		{
			auto it = m_timeScale.find(id);
			if (it == m_timeScale.end()) return;
			if (it->second.s <= 0.0f) Thaw(b, soft, it->second);
			b.GetMotionProperties()->SetGravityFactor(1.0f);
			m_timeScale.erase(it);
			VehicleTimeScale(id, 1.0f);
			return;
		}
		TimeScaleRec& r = m_timeScale[id];
		if (r.s <= 0.0f && s > 0.0f) Thaw(b, soft, r);
		r.s = s;
		b.GetMotionProperties()->SetGravityFactor(s * s);   // x += v s dt with v scaled by s: gravity must add g s dt -> factor s^2
		VehicleTimeScale(id, s);
	}
	void setBodyFriction(uint64_t handle, float friction, float restitution) override
	{
		if (!m_system || !handle) return;
		const JPH::uint32 id = (JPH::uint32)handle;
		if (!m_bodies.count(id)) return;
		JPH::BodyInterface& bi = m_system->GetBodyInterface();
		bi.SetFriction(JPH::BodyID(id), std::max(friction, 0.0f));
		bi.SetRestitution(JPH::BodyID(id), std::min(std::max(restitution, 0.0f), 1.0f));
	}
	template <class Rec>   // TimeScaleRec (declared below; a parameter list can't see it yet)
	static void Thaw(JPH::Body& b, JPH::SoftBodyMotionProperties* soft, const Rec& r)
	{
		if (soft)
		{
			JPH::Array<JPH::SoftBodyVertex>& vs = soft->GetVertices();
			for (size_t i = 0; i < vs.size() && i < r.vv.size(); ++i) vs[i].mVelocity = r.vv[i];
			return;
		}
		b.SetLinearVelocity(r.v); b.SetAngularVelocity(r.w);
	}
	// Around one solve: v -> v s (frozen bodies park their velocity and stand still), then back.
	// Kinematic bodies are skipped: their driver already moved them by a scaled displacement.
	void TimeScalePre()
	{
		if (m_timeScale.empty()) return;
		for (auto& kv : m_timeScale)
		{
			if (!m_bodies.count(kv.first)) continue;
			JPH::BodyLockWrite lock(m_system->GetBodyLockInterface(), JPH::BodyID(kv.first));
			if (!lock.Succeeded()) continue;
			JPH::Body& b = lock.GetBody();
			if (!b.IsDynamic() && !b.IsSoftBody()) continue;
			TimeScaleRec& r = kv.second;
			JPH::SoftBodyMotionProperties* soft = SoftMP(b);
			if (r.s <= 0.0f)
			{
				if (soft)
				{
					JPH::Array<JPH::SoftBodyVertex>& vs = soft->GetVertices();
					bool moving = false;
					for (const JPH::SoftBodyVertex& v : vs) if (v.mVelocity.LengthSq() > 0.0f) { moving = true; break; }
					if (moving) { r.vv.resize(vs.size()); for (size_t i = 0; i < vs.size(); ++i) r.vv[i] = vs[i].mVelocity; }
					for (JPH::SoftBodyVertex& v : vs) v.mVelocity = JPH::Vec3::sZero();
				}
				else
				{
					const JPH::Vec3 v = b.GetLinearVelocity(), w = b.GetAngularVelocity();
					if (v.LengthSq() > 0.0f || w.LengthSq() > 0.0f) { r.v = v; r.w = w; }   // a push while frozen is dropped
					b.SetLinearVelocity(JPH::Vec3::sZero()); b.SetAngularVelocity(JPH::Vec3::sZero());
				}
				b.GetMotionProperties()->SetGravityFactor(0.0f);
			}
			else if (soft)
			{
				for (JPH::SoftBodyVertex& v : soft->GetVertices()) v.mVelocity *= r.s;
			}
			else
			{
				b.SetLinearVelocity(b.GetLinearVelocity() * r.s);
				b.SetAngularVelocity(b.GetAngularVelocity() * r.s);
			}
		}
	}
	void TimeScalePost()
	{
		if (m_timeScale.empty()) return;
		for (auto& kv : m_timeScale)
		{
			const TimeScaleRec& r = kv.second;
			if (!m_bodies.count(kv.first) || r.s <= 0.0f) continue;
			JPH::BodyLockWrite lock(m_system->GetBodyLockInterface(), JPH::BodyID(kv.first));
			if (!lock.Succeeded()) continue;
			JPH::Body& b = lock.GetBody();
			if (!b.IsDynamic() && !b.IsSoftBody()) continue;
			if (JPH::SoftBodyMotionProperties* soft = SoftMP(b))
			{
				for (JPH::SoftBodyVertex& v : soft->GetVertices()) v.mVelocity /= r.s;
				b.SetLinearVelocity(b.GetLinearVelocity() / r.s);   // the summary velocity follows
				continue;
			}
			b.SetLinearVelocity(b.GetLinearVelocity() / r.s);
			b.SetAngularVelocity(b.GetAngularVelocity() / r.s);
		}
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

	void addForceAtPoint(uint64_t handle, const float f[3], const float p[3]) override
	{
		if (!m_system || !handle) return;
		m_system->GetBodyInterface().AddForce(JPH::BodyID((JPH::uint32)handle),
		                                      JPH::Vec3(f[0], f[1], f[2]), JPH::RVec3(p[0], p[1], p[2]));
	}

	void getPointVelocity(uint64_t handle, const float p[3], float outVel[3]) override
	{
		outVel[0] = outVel[1] = outVel[2] = 0.0f;
		if (!m_system || !handle) return;
		JPH::Vec3 v = m_system->GetBodyInterface().GetPointVelocity(JPH::BodyID((JPH::uint32)handle),
		                                                            JPH::RVec3(p[0], p[1], p[2]));
		outVel[0] = v.GetX(); outVel[1] = v.GetY(); outVel[2] = v.GetZ();
	}

	// ---- ragdoll joints (stage 9) ---------------------------------------------------------
	uint64_t createSwingTwistJoint(const NukeJointDesc& d) override
	{
		if (!m_system || !d.bodyA || !d.bodyB) return 0;
		const JPH::BodyID ids[2] = { JPH::BodyID((JPH::uint32)d.bodyA), JPH::BodyID((JPH::uint32)d.bodyB) };
		JPH::BodyLockMultiWrite lock(m_system->GetBodyLockInterface(), ids, 2);
		JPH::Body* a = lock.GetBody(0);
		JPH::Body* b = lock.GetBody(1);
		if (!a || !b) return 0;
		JPH::SwingTwistConstraintSettings st;
		st.mSpace = JPH::EConstraintSpace::WorldSpace;
		st.mPosition1 = st.mPosition2 = JPH::RVec3(d.pivot[0], d.pivot[1], d.pivot[2]);
		JPH::Vec3 tw(d.twistAxis[0], d.twistAxis[1], d.twistAxis[2]);
		JPH::Vec3 pl(d.planeAxis[0], d.planeAxis[1], d.planeAxis[2]);
		if (tw.LengthSq() < 1e-10f) tw = JPH::Vec3::sAxisY();
		tw = tw.Normalized();
		pl -= tw * pl.Dot(tw);   // enforce perpendicularity
		if (pl.LengthSq() < 1e-10f) pl = tw.GetNormalizedPerpendicular();
		pl = pl.Normalized();
		st.mTwistAxis1 = st.mTwistAxis2 = tw;
		st.mPlaneAxis1 = st.mPlaneAxis2 = pl;
		st.mPlaneHalfConeAngle  = d.swing1;
		st.mNormalHalfConeAngle = d.swing2;
		st.mTwistMinAngle = d.twistMin;
		st.mTwistMaxAngle = d.twistMax;
		JPH::SwingTwistConstraint* c =
			static_cast<JPH::SwingTwistConstraint*>(st.Create(*a, *b));
		m_system->AddConstraint(c);
		const uint64_t id = m_nextJoint++;
		m_joints[id] = c;
		return id;
	}

	void destroyJoint(uint64_t joint) override
	{
		if (!m_system) return;
		auto it = m_joints.find(joint);
		if (it != m_joints.end())
		{
			m_system->RemoveConstraint(it->second);
			m_joints.erase(it);
			return;
		}
		auto ic = m_constraints.find(joint);
		if (ic != m_constraints.end())
		{
			m_system->RemoveConstraint(ic->second.first);
			m_constraints.erase(ic);
		}
	}

	void setJointMotor(uint64_t joint, bool enabled, float frequency, float damping) override
	{
		auto it = m_joints.find(joint);
		if (it == m_joints.end()) return;
		JPH::SwingTwistConstraint* c = it->second;
		const JPH::EMotorState s = enabled ? JPH::EMotorState::Position : JPH::EMotorState::Off;
		JPH::MotorSettings& sm = c->GetSwingMotorSettings();
		JPH::MotorSettings& tm = c->GetTwistMotorSettings();
		sm.mSpringSettings.mFrequency = frequency;
		sm.mSpringSettings.mDamping = damping;
		tm.mSpringSettings.mFrequency = frequency;
		tm.mSpringSettings.mDamping = damping;
		c->SetSwingMotorState(s);
		c->SetTwistMotorState(s);
	}

	void setJointTarget(uint64_t joint, const float q[4]) override
	{
		auto it = m_joints.find(joint);
		if (it == m_joints.end()) return;
		it->second->SetTargetOrientationBS(JPH::Quat(q[0], q[1], q[2], q[3]).Normalized());
	}

	// ---- generic constraints: appended at the iPhysics vtable END (ABI) ----

	uint64_t createConstraint(const NukeConstraintDesc& d) override
	{
		if (!m_system || !d.bodyB) return 0;
		const JPH::BodyID idA((JPH::uint32)d.bodyA);
		const JPH::BodyID idB((JPH::uint32)d.bodyB);
		JPH::Constraint* c = nullptr;
		if (d.bodyA)
		{
			// Never MultiWrite-lock a duplicated id — same-mutex relock is undefined.
			const JPH::BodyID ids[2] = { idA, idB };
			JPH::BodyLockMultiWrite lock(m_system->GetBodyLockInterface(), ids, 2);
			JPH::Body* a = lock.GetBody(0);
			JPH::Body* b = lock.GetBody(1);
			if (!a || !b) return 0;
			c = BuildConstraint(d, *a, *b);
		}
		else
		{
			JPH::BodyLockWrite lock(m_system->GetBodyLockInterface(), idB);
			if (!lock.Succeeded()) return 0;
			c = BuildConstraint(d, JPH::Body::sFixedToWorld, lock.GetBody());
		}
		if (!c) return 0;
		m_system->AddConstraint(c);
		// A constraint changes the equilibrium — wake what it grabs (a sleeping body would
		// hang mid-air until something else touches it).
		JPH::BodyInterface& bi = m_system->GetBodyInterface();
		bi.ActivateBody(idB);
		if (d.bodyA) bi.ActivateBody(idA);
		const uint64_t id = m_nextJoint++;
		m_constraints[id] = { static_cast<JPH::TwoBodyConstraint*>(c), d.type };
		return id;
	}

	static JPH::Constraint* BuildConstraint(const NukeConstraintDesc& d, JPH::Body& a, JPH::Body& b)
	{
		const JPH::RVec3 pivot(d.pivot[0], d.pivot[1], d.pivot[2]);
		const JPH::RVec3 pivotB(d.pivotB[0], d.pivotB[1], d.pivotB[2]);
		JPH::Vec3 axis(d.axis[0], d.axis[1], d.axis[2]);
		if (axis.LengthSq() < 1e-10f) axis = JPH::Vec3::sAxisZ();
		axis = axis.Normalized();
		JPH::Vec3 nrm(d.normal[0], d.normal[1], d.normal[2]);
		nrm -= axis * nrm.Dot(axis);   // enforce perpendicularity
		if (nrm.LengthSq() < 1e-10f) nrm = axis.GetNormalizedPerpendicular();
		nrm = nrm.Normalized();
		JPH::Constraint* c = nullptr;
		switch (d.type)
		{
			case 0:
			{
				JPH::HingeConstraintSettings s;
				s.mSpace = JPH::EConstraintSpace::WorldSpace;
				s.mPoint1 = s.mPoint2 = pivot;
				s.mHingeAxis1 = s.mHingeAxis2 = axis;
				s.mNormalAxis1 = s.mNormalAxis2 = nrm;
				if (d.limit) { s.mLimitsMin = d.min; s.mLimitsMax = d.max; }
				c = s.Create(a, b);
				break;
			}
			case 1:
			{
				JPH::SliderConstraintSettings s;
				s.mSpace = JPH::EConstraintSpace::WorldSpace;
				s.mPoint1 = s.mPoint2 = pivot;
				s.mSliderAxis1 = s.mSliderAxis2 = axis;
				s.mNormalAxis1 = s.mNormalAxis2 = nrm;
				if (d.limit) { s.mLimitsMin = d.min; s.mLimitsMax = d.max; }
				c = s.Create(a, b);
				break;
			}
			case 2:
			case 3:
			{
				JPH::DistanceConstraintSettings s;
				s.mSpace = JPH::EConstraintSpace::WorldSpace;
				s.mPoint1 = pivot;
				s.mPoint2 = pivotB;
				if (d.limit) { s.mMinDistance = d.min; s.mMaxDistance = d.max; }
				if (d.frequency > 0.0f)
					s.mLimitsSpringSettings = JPH::SpringSettings(JPH::ESpringMode::FrequencyAndDamping,
					                                              d.frequency, d.damping);
				c = s.Create(a, b);
				break;
			}
			case 4:
			{
				JPH::ConeConstraintSettings s;
				s.mSpace = JPH::EConstraintSpace::WorldSpace;
				s.mPoint1 = s.mPoint2 = pivot;
				s.mTwistAxis1 = s.mTwistAxis2 = axis;
				s.mHalfConeAngle = d.halfCone;
				c = s.Create(a, b);
				break;
			}
		}
		return c;
	}

	void setConstraintMotor(uint64_t cid, int mode, float target, float maxForce) override
	{
		auto it = m_constraints.find(cid);
		if (it == m_constraints.end()) return;
		const JPH::EMotorState st = mode == 1 ? JPH::EMotorState::Velocity
		                          : mode == 2 ? JPH::EMotorState::Position
		                                      : JPH::EMotorState::Off;
		if (it->second.second == 0)
		{
			auto* h = static_cast<JPH::HingeConstraint*>(it->second.first);
			JPH::MotorSettings& ms = h->GetMotorSettings();
			if (maxForce > 0.0f) { ms.mMinTorqueLimit = -maxForce; ms.mMaxTorqueLimit = maxForce; }
			h->SetMotorState(st);
			if (mode == 1) h->SetTargetAngularVelocity(target);
			else if (mode == 2) h->SetTargetAngle(target);
		}
		else if (it->second.second == 1)
		{
			auto* s = static_cast<JPH::SliderConstraint*>(it->second.first);
			JPH::MotorSettings& ms = s->GetMotorSettings();
			if (maxForce > 0.0f) { ms.mMinForceLimit = -maxForce; ms.mMaxForceLimit = maxForce; }
			s->SetMotorState(st);
			if (mode == 1) s->SetTargetVelocity(target);
			else if (mode == 2) s->SetTargetPosition(target);
		}
		else
			return;
		// A sleeping body ignores its motor until something wakes it.
		if (JPH::Body* b2 = it->second.first->GetBody2())
			m_system->GetBodyInterface().ActivateBody(b2->GetID());
	}

	float constraintImpulse(uint64_t cid) override
	{
		auto it = m_constraints.find(cid);
		if (it == m_constraints.end()) return 0.0f;
		JPH::TwoBodyConstraint* c = it->second.first;
		switch (it->second.second)
		{
			case 0: return static_cast<JPH::HingeConstraint*>(c)->GetTotalLambdaPosition().Length();
			case 1:
			{
				const auto l = static_cast<JPH::SliderConstraint*>(c)->GetTotalLambdaPosition();
				return std::sqrt(l[0] * l[0] + l[1] * l[1]);
			}
			case 2:
			case 3: return std::fabs(static_cast<JPH::DistanceConstraint*>(c)->GetTotalLambdaPosition());
			case 4: return static_cast<JPH::ConeConstraint*>(c)->GetTotalLambdaPosition().Length();
		}
		return 0.0f;
	}

	// ---- wheeled vehicles: appended at the iPhysics vtable END (ABI) ----

	uint64_t createVehicle(const NukeVehicleDesc& d) override
	{
		if (!m_system || !d.chassis || !d.wheels || d.wheelCount < 1) return 0;
		if (d.type == 2) return CreateHover(d);
		if (d.type == 1) return CreateTracked(d);
		JPH::Ref<JPH::VehicleConstraint> vc;
		VehicleBase base;
		{
			JPH::BodyLockWrite lock(m_system->GetBodyLockInterface(), JPH::BodyID((JPH::uint32)d.chassis));
			if (!lock.Succeeded()) return 0;
			JPH::VehicleConstraintSettings vs;
			auto* controller = new JPH::WheeledVehicleControllerSettings();
			controller->mEngine.mMaxTorque = d.maxTorque;
			base.maxTorque = d.maxTorque;
			controller->mEngine.mMaxRPM = d.maxRPM;
			int prevDriven = -1;
			for (int i = 0; i < d.wheelCount; ++i)
			{
				const NukeWheelDesc& w = d.wheels[i];
				auto* ws = new JPH::WheelSettingsWV();
				ws->mPosition = JPH::Vec3(w.pos[0], w.pos[1], w.pos[2]);
				ws->mRadius = w.radius;
				ws->mWidth = w.width;
				ws->mSuspensionMinLength = w.suspensionMin;
				ws->mSuspensionMaxLength = std::max(w.suspensionMax, w.suspensionMin + 0.01f);
				ws->mSuspensionSpring = JPH::SpringSettings(JPH::ESpringMode::FrequencyAndDamping,
				                                            w.frequency, w.damping);
				ws->mMaxSteerAngle = w.maxSteerDeg * 0.01745329252f;
				ws->mMaxBrakeTorque = w.maxBrakeTorque;
				ws->mMaxHandBrakeTorque = w.maxHandBrakeTorque;
				base.wheels.push_back({ ws, w.frequency, w.maxBrakeTorque, w.maxHandBrakeTorque });
				vs.mWheels.push_back(ws);
				if (w.driven)
				{
					// Driven wheels pair into differentials in declaration order.
					if (prevDriven < 0) prevDriven = i;
					else
					{
						JPH::VehicleDifferentialSettings ds;
						ds.mLeftWheel = prevDriven;
						ds.mRightWheel = i;
						controller->mDifferentials.push_back(ds);
						prevDriven = -1;
					}
				}
			}
			if (prevDriven >= 0)   // odd driven wheel drives alone
			{
				JPH::VehicleDifferentialSettings ds;
				ds.mLeftWheel = prevDriven;
				ds.mRightWheel = -1;
				controller->mDifferentials.push_back(ds);
			}
			if (!controller->mDifferentials.empty())
				for (JPH::VehicleDifferentialSettings& ds : controller->mDifferentials)
					ds.mEngineTorqueRatio = 1.0f / controller->mDifferentials.size();
			vs.mController = controller;
			vc = new JPH::VehicleConstraint(lock.GetBody(), vs);
			vc->SetVehicleCollisionTester(new JPH::VehicleCollisionTesterCastCylinder(ObjLayers::MOVING));
		}
		m_system->AddConstraint(vc);
		m_system->AddStepListener(vc);
		m_system->GetBodyInterface().ActivateBody(JPH::BodyID((JPH::uint32)d.chassis));
		const uint64_t id = m_nextJoint++;
		m_vehicles[id] = vc;
		m_vehBase[id] = base;
		// A chassis already living in local time: the car takes it from the start.
		auto ts = m_timeScale.find((JPH::uint32)d.chassis);
		if (ts != m_timeScale.end()) VehicleTimeScale((JPH::uint32)d.chassis, ts->second.s);
		return id;
	}

	// Tracked: Jolt's TrackedVehicleController - the road wheels split into a left and a right
	// track by side, the engine drives both through the gear box, steering scales each track's
	// rate (setVehicleInput). Every track needs at least one wheel.
	uint64_t CreateTracked(const NukeVehicleDesc& d)
	{
		JPH::Ref<JPH::VehicleConstraint> vc;
		VehicleBase base;
		base.type = 1;
		{
			JPH::BodyLockWrite lock(m_system->GetBodyLockInterface(), JPH::BodyID((JPH::uint32)d.chassis));
			if (!lock.Succeeded()) return 0;
			JPH::VehicleConstraintSettings vs;
			auto* controller = new JPH::TrackedVehicleControllerSettings();
			controller->mEngine.mMaxTorque = d.maxTorque;
			controller->mEngine.mMaxRPM = d.maxRPM;
			base.maxTorque = d.maxTorque;
			base.trackBrake = d.trackBrakeTorque;
			int driven[2] = { -1, -1 };
			for (int i = 0; i < d.wheelCount; ++i)
			{
				const NukeWheelDesc& w = d.wheels[i];
				auto* ws = new JPH::WheelSettingsTV();
				ws->mPosition = JPH::Vec3(w.pos[0], w.pos[1], w.pos[2]);
				ws->mRadius = w.radius;
				ws->mWidth = w.width;
				ws->mSuspensionMinLength = w.suspensionMin;
				ws->mSuspensionMaxLength = std::max(w.suspensionMax, w.suspensionMin + 0.01f);
				ws->mSuspensionSpring = JPH::SpringSettings(JPH::ESpringMode::FrequencyAndDamping, w.frequency, w.damping);
				ws->mLongitudinalFriction = w.longFriction;
				ws->mLateralFriction = w.latFriction;
				base.wheels.push_back({ ws, w.frequency, 0.0f, 0.0f });
				vs.mWheels.push_back(ws);
				// Jolt's track 0 ("Left") is the +X side (forward +Z, right-handed: its right is -X); the
				// engine's left is -X, so the engine-left wheels go to track 1 - then the steering below
				// turns a tank the way it turns a car (stand-verified against a wheeled one).
				const bool engineLeft = w.side >= 0 ? (w.side == 0) : (w.pos[0] < 0.0f);
				const int side = engineLeft ? 1 : 0;
				controller->mTracks[side].mWheels.push_back((JPH::uint)i);
				if (driven[side] < 0 || (w.driven && !d.wheels[driven[side]].driven)) driven[side] = i;
			}
			if (driven[0] < 0 || driven[1] < 0)
			{
				for (JPH::WheelSettings* ws : vs.mWheels) delete ws;
				delete controller;
				return 0;   // a track without wheels
			}
			for (int t = 0; t < 2; ++t)
			{
				JPH::VehicleTrackSettings& tr = controller->mTracks[t];
				tr.mDrivenWheel = (JPH::uint)driven[t];
				tr.mInertia = d.trackInertia;
				tr.mAngularDamping = d.trackDamping;
				tr.mMaxBrakeTorque = d.trackBrakeTorque;
				tr.mDifferentialRatio = d.trackDiffRatio;
			}
			vs.mController = controller;
			vc = new JPH::VehicleConstraint(lock.GetBody(), vs);
			vc->SetVehicleCollisionTester(new JPH::VehicleCollisionTesterCastCylinder(ObjLayers::MOVING));
		}
		m_system->AddConstraint(vc);
		m_system->AddStepListener(vc);
		m_system->GetBodyInterface().ActivateBody(JPH::BodyID((JPH::uint32)d.chassis));
		const uint64_t id = m_nextJoint++;
		m_vehicles[id] = vc;
		m_vehBase[id] = base;
		auto ts = m_timeScale.find((JPH::uint32)d.chassis);
		if (ts != m_timeScale.end()) VehicleTimeScale((JPH::uint32)d.chassis, ts->second.s);
		return id;
	}

	// Hover: no constraint. The thrusters are springs toward their hover height over whatever
	// a ray down the chassis finds (StepHovers, a step listener); thrust / turn / grip / brake /
	// self-righting act at the chassis while any thruster has ground under it.
	uint64_t CreateHover(const NukeVehicleDesc& d)
	{
		HoverRec h;
		h.body = (JPH::uint32)d.chassis;
		if (!m_bodies.count(h.body)) return 0;
		for (int i = 0; i < d.wheelCount; ++i)
		{
			const NukeWheelDesc& w = d.wheels[i];
			HoverRec::Thruster t;
			t.local = JPH::Vec3(w.pos[0], w.pos[1], w.pos[2]);
			t.height = std::max(w.suspensionMax, 0.05f);
			t.frequency = std::max(w.frequency, 0.1f);
			t.damping = std::max(w.damping, 0.0f);
			t.length = t.height;
			h.thrusters.push_back(t);
		}
		h.thrust = d.hoverThrust; h.turn = d.hoverTurn; h.grip = d.hoverGrip;
		h.upright = d.hoverUpright; h.angDamp = d.hoverAngularDamping; h.brakeForce = d.hoverBrake;
		auto ts = m_timeScale.find(h.body);
		if (ts != m_timeScale.end()) h.s = ts->second.s;
		if (!m_hoverStepper.added)
		{
			m_hoverStepper.self = this;
			m_system->AddStepListener(&m_hoverStepper);
			m_hoverStepper.added = true;
		}
		m_system->GetBodyInterface().ActivateBody(JPH::BodyID(h.body));
		const uint64_t id = m_nextJoint++;
		m_hovers[id] = h;
		return id;
	}

	// Before every physics step: lift springs + drive forces of every hover. The step holds the
	// body mutexes already (NoLock interfaces), and the chassis is skipped when asleep.
	void StepHovers(const JPH::PhysicsStepListenerContext& ctx)
	{
		JPH::PhysicsSystem* sys = ctx.mPhysicsSystem;
		const float dt = ctx.mDeltaTime;
		for (auto& kv : m_hovers)
		{
			HoverRec& h = kv.second;
			JPH::BodyLockWrite lock(sys->GetBodyLockInterfaceNoLock(), JPH::BodyID(h.body));
			if (!lock.Succeeded()) continue;
			JPH::Body& b = lock.GetBody();
			if (!b.IsActive() || !b.IsDynamic()) continue;
			const float invM = b.GetMotionProperties()->GetInverseMass();
			if (invM <= 0.0f) continue;
			const float mass = 1.0f / invM, s = h.s, s2 = s * s;
			const JPH::RMat44 xf = b.GetWorldTransform();
			const JPH::Vec3 up = xf.GetAxisY(), fwd = xf.GetAxisZ();
			const float share = mass / (float)std::max((int)h.thrusters.size(), 1);
			bool grounded = false;
			JPH::IgnoreSingleBodyFilter ignore{ b.GetID() };
			for (HoverRec::Thruster& t : h.thrusters)
			{
				const JPH::RVec3 origin = xf * t.local;
				const float castLen = t.height * 1.5f + 0.01f;   // see the ground a little past the hover height
				JPH::RRayCast ray{ origin, -up * castLen };
				JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> col;
				sys->GetNarrowPhaseQueryNoLock().CastRay(ray, JPH::RayCastSettings{}, col, {}, {}, ignore);
				t.contact = col.HadHit();
				t.length = t.contact ? col.mHit.mFraction * castLen : castLen;
				if (!t.contact || t.length >= t.height) continue;
				grounded = true;
				const float w = 6.2831853f * t.frequency * s;
				const float k = share * w * w, c = 2.0f * t.damping * std::sqrt(k * share);
				const float vAlong = b.GetPointVelocity(origin).Dot(up);   // + = rising
				const float force = std::max(k * (t.height - t.length) - c * vAlong, 0.0f);   // a spring that only pushes
				b.AddForce(up * force, origin);
			}
			if (!grounded) continue;
			const JPH::Vec3 vel = b.GetLinearVelocity();
			const JPH::Vec3 right = fwd.Cross(up);
			const float vF = vel.Dot(fwd), vR = vel.Dot(right);
			JPH::Vec3 force = fwd * (h.inF * h.thrust * s2);
			if (h.inB > 0.0f && std::fabs(vF) > 1e-3f)   // brake: against the forward motion, never past a stop
				force -= fwd * std::min(h.inB * h.brakeForce * s2, std::fabs(vF) * mass / std::max(dt, 1e-4f)) * (vF > 0.0f ? 1.0f : -1.0f);
			force -= right * std::min(h.grip * s, 1.0f / std::max(dt, 1e-4f)) * vR * mass;   // side grip
			b.AddForce(force);
			const JPH::Vec3 av = b.GetAngularVelocity();
			JPH::Vec3 torque = up * (h.inR * h.turn * s2);
			torque += up.Cross(JPH::Vec3::sAxisY()) * (h.upright * s2 * mass);   // self-righting toward world up
			torque -= av * (std::min(h.angDamp * s, 1.0f / std::max(dt, 1e-4f)) * mass);
			b.AddTorque(torque);
		}
	}

	void destroyVehicle(uint64_t v) override
	{
		if (!m_system) return;
		if (m_hovers.erase(v)) return;
		auto it = m_vehicles.find(v);
		if (it == m_vehicles.end()) return;
		m_system->RemoveStepListener(it->second);
		m_system->RemoveConstraint(it->second);
		m_vehicles.erase(it);
		m_vehBase.erase(v);
	}

	void setVehicleInput(uint64_t v, float forward, float right, float brake, float handBrake) override
	{
		const bool any = std::fabs(forward) > 0.01f || std::fabs(right) > 0.01f || brake > 0.01f || handBrake > 0.01f;
		auto hit = m_hovers.find(v);
		if (hit != m_hovers.end())
		{
			HoverRec& h = hit->second;
			h.inF = std::clamp(forward, -1.0f, 1.0f); h.inR = std::clamp(right, -1.0f, 1.0f);
			h.inB = std::clamp(std::max(brake, handBrake), 0.0f, 1.0f);
			if (any) m_system->GetBodyInterface().ActivateBody(JPH::BodyID(h.body));
			return;
		}
		auto it = m_vehicles.find(v);
		if (it == m_vehicles.end()) return;
		auto bit = m_vehBase.find(v);
		if (bit != m_vehBase.end() && bit->second.type == 1)
		{
			// Tracks: steer slows the inner track down to counter-rotation at full lock; with no
			// throttle the two tracks counter-rotate and the vehicle pivots in place.
			float f = forward, l = 1.0f, r = 1.0f;
			if (std::fabs(f) < 0.05f && std::fabs(right) > 0.05f)
			{
				f = std::fabs(right);
				l = right > 0.0f ? 1.0f : -1.0f; r = -l;
			}
			else
			{
				const float inner = std::max(1.0f - std::fabs(right) * 1.5f, -1.0f);
				if (right > 0.0f) r = inner; else if (right < 0.0f) l = inner;
			}
			if (std::fabs(l) < 0.01f) l = l < 0.0f ? -0.01f : 0.01f;   // Jolt: a ratio is never 0
			if (std::fabs(r) < 0.01f) r = r < 0.0f ? -0.01f : 0.01f;
			static_cast<JPH::TrackedVehicleController*>(it->second->GetController())
				->SetDriverInput(f, l, r, std::max(brake, handBrake));
		}
		else
		{
			auto* c = static_cast<JPH::WheeledVehicleController*>(it->second->GetController());
			c->SetDriverInput(forward, right, brake, handBrake);
		}
		if (any) m_system->GetBodyInterface().ActivateBody(it->second->GetVehicleBody()->GetID());
	}

	bool getWheelState(uint64_t v, int wheel, NukeWheelState& out) override
	{
		auto hit = m_hovers.find(v);
		if (hit != m_hovers.end())
		{
			const HoverRec& h = hit->second;
			if (wheel < 0 || wheel >= (int)h.thrusters.size()) return false;
			JPH::BodyLockRead lock(m_system->GetBodyLockInterface(), JPH::BodyID(h.body));
			if (!lock.Succeeded()) return false;
			const HoverRec::Thruster& t = h.thrusters[wheel];
			const JPH::RVec3 p = lock.GetBody().GetWorldTransform() * t.local;
			const JPH::Quat q = lock.GetBody().GetRotation();
			out.pos[0] = (float)p.GetX(); out.pos[1] = (float)p.GetY(); out.pos[2] = (float)p.GetZ();
			out.quat[0] = q.GetX(); out.quat[1] = q.GetY(); out.quat[2] = q.GetZ(); out.quat[3] = q.GetW();
			out.suspension = t.length;   // current height over the ground (hover height when nothing is in reach)
			out.contact = t.contact ? 1 : 0;
			out.longSlip = out.latSlip = 0.0f;
			return true;
		}
		auto it = m_vehicles.find(v);
		if (it == m_vehicles.end()) return false;
		JPH::VehicleConstraint* c = it->second;
		if (wheel < 0 || wheel >= (int)c->GetWheels().size()) return false;
		const JPH::RMat44 m = c->GetWheelWorldTransform((JPH::uint)wheel, JPH::Vec3::sAxisY(), JPH::Vec3::sAxisX());
		const JPH::RVec3 p = m.GetTranslation();
		const JPH::Quat q = m.GetQuaternion().Normalized();
		out.pos[0] = (float)p.GetX(); out.pos[1] = (float)p.GetY(); out.pos[2] = (float)p.GetZ();
		out.quat[0] = q.GetX(); out.quat[1] = q.GetY(); out.quat[2] = q.GetZ(); out.quat[3] = q.GetW();
		const JPH::Wheel* w = c->GetWheel((JPH::uint)wheel);
		out.suspension = w->GetSuspensionLength();
		out.contact = w->HasContact() ? 1 : 0;
		auto bit = m_vehBase.find(v);
		if (bit != m_vehBase.end() && bit->second.type == 1)
		{
			out.longSlip = out.latSlip = 0.0f;   // a track pad: Jolt reports no slip for it
		}
		else
		{
			const auto* wv = static_cast<const JPH::WheelWV*>(w);
			out.longSlip = wv->mLongitudinalSlip;
			out.latSlip = wv->mLateralSlip;
		}
		return true;
	}

	float vehicleRPM(uint64_t v) override
	{
		if (m_hovers.count(v)) return 0.0f;   // no engine model: thrust is the input itself
		auto it = m_vehicles.find(v);
		if (it == m_vehicles.end()) return 0.0f;
		auto bit = m_vehBase.find(v);
		if (bit != m_vehBase.end() && bit->second.type == 1)
			return static_cast<JPH::TrackedVehicleController*>(it->second->GetController())->GetEngine().GetCurrentRPM();
		return static_cast<JPH::WheeledVehicleController*>(it->second->GetController())->GetEngine().GetCurrentRPM();
	}

	float vehicleSpeed(uint64_t v) override
	{
		JPH::BodyID id;
		auto hit = m_hovers.find(v);
		if (hit != m_hovers.end()) id = JPH::BodyID(hit->second.body);
		else
		{
			auto it = m_vehicles.find(v);
			if (it == m_vehicles.end()) return 0.0f;
			id = it->second->GetVehicleBody()->GetID();
		}
		JPH::BodyLockRead lock(m_system->GetBodyLockInterface(), id);
		if (!lock.Succeeded()) return 0.0f;
		const JPH::Body& body = lock.GetBody();
		return body.GetRotation().RotateAxisZ().Dot(body.GetLinearVelocity());
	}

	bool sphereCastDist(float radius, const float from[3], const float dir[3],
	                    float maxDist, uint64_t ignoreBody, float& outDist) override
	{
		if (!m_system || maxDist <= 0.0f) return false;
		JPH::Vec3 d(dir[0], dir[1], dir[2]);
		if (d.LengthSq() < 1e-12f) return false;
		d = d.Normalized() * maxDist;
		JPH::RefConst<JPH::Shape> shape = new JPH::SphereShape(std::max(1e-4f, radius));
		JPH::RShapeCast cast(shape, JPH::Vec3::sReplicate(1.0f),
		                     JPH::RMat44::sTranslation(JPH::RVec3(from[0], from[1], from[2])), d);
		JPH::ShapeCastSettings settings;
		JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
		JPH::IgnoreSingleBodyFilter ignore{ JPH::BodyID((JPH::uint32)ignoreBody) };
		m_system->GetNarrowPhaseQuery().CastShape(cast, settings, JPH::RVec3::sZero(), collector,
		                                          {}, {}, ignore);
		if (!collector.HadHit()) return false;
		outDist = collector.mHit.mFraction * maxDist;
		return true;
	}

	// ---- isolated scenes: appended at the iPhysics vtable END (ABI 35) ----
	// Each JoltPhysics instance is a self-contained scene (own PhysicsSystem/JobSystem/
	// TempAllocator); the Jolt process-globals (allocator, Factory, Trace) are guarded
	// once in init(). Deleting a scene must NOT run shutdown() — that tears down the
	// global Factory under every other scene.

	iPhysics* createScene() override { return new JoltPhysics(); }

	void destroyScene(iPhysics* s) override
	{
		if (!s || s == this) return;   // never the service's own main scene
		JoltPhysics* jp = static_cast<JoltPhysics*>(s);
		jp->reset();                   // drop bodies/characters/joints while the system lives
		if (jp->m_system) jp->step(1.0f / 60.0f);   // broad phase frees removed nodes lazily
		delete jp;
	}

	// ---- soft-body cloth (C3): appended at the iPhysics vtable END (ABI 40) ----
	// A soft body is a regular Jolt body (handle-compatible with destroyBody's sweep in
	// reset()); rotation stays identity and mUpdatePosition is off, so the body-local <->
	// world conversion is a fixed translation for the body's whole life.

	uint64_t createSoftBody(const NukeSoftBodyDesc& d) override
	{
		if (!m_system || !d.verts || d.numVerts < 3 || !d.indices || d.numTris < 1) return 0;
		using SBSS = JPH::SoftBodySharedSettings;
		JPH::Ref<SBSS> ss = new SBSS();
		ss->mVertices.reserve(d.numVerts);
		for (int i = 0; i < d.numVerts; ++i)
		{
			SBSS::Vertex v;
			v.mPosition = JPH::Float3(d.verts[i * 3 + 0] - d.pos[0],
			                          d.verts[i * 3 + 1] - d.pos[1],
			                          d.verts[i * 3 + 2] - d.pos[2]);
			v.mInvMass = d.invMass ? d.invMass[i] : 1.0f;
			ss->mVertices.push_back(v);
		}
		int faces = 0;
		for (int t = 0; t < d.numTris; ++t)
		{
			SBSS::Face f(d.indices[t * 3 + 0], d.indices[t * 3 + 1], d.indices[t * 3 + 2]);
			if (f.IsDegenerate()) continue;
			ss->AddFace(f);
			++faces;
		}
		if (faces == 0) return 0;
		const SBSS::VertexAttributes attr(d.compliance, d.compliance, d.bendCompliance);
		ss->CreateConstraints(&attr, 1, SBSS::EBendType::Dihedral);

		// Skinned-constraint block (fitted clothes): the solver itself leashes every vertex
		// to its skinned position with a face-normal backstop behind the surface.
		if (d.invBind && d.numJoints > 0 && d.skinJoints && d.skinWeights)
		{
			ss->mInvBindMatrices.reserve(d.numJoints);
			for (int j = 0; j < d.numJoints; ++j)
				ss->mInvBindMatrices.push_back(SBSS::InvBind(
					(JPH::uint32)j, JPH::Mat44::sLoadFloat4x4((const JPH::Float4*)(d.invBind + j * 16))));
			ss->mSkinnedConstraints.reserve(d.numVerts);
			for (int v = 0; v < d.numVerts; ++v)
			{
				const float maxDist = d.skinMaxDist ? d.skinMaxDist[v] : FLT_MAX;
				SBSS::Skinned sk((JPH::uint32)v,
				                 maxDist >= 1e9f ? FLT_MAX : maxDist,
				                 d.backstopDistance >= 1e9f ? FLT_MAX : d.backstopDistance,
				                 d.backstopRadius);
				int w = 0;
				for (int k = 0; k < 4 && w < (int)SBSS::Skinned::cMaxSkinWeights; ++k)
				{
					const float wt = d.skinWeights[v * 4 + k];
					const int j = d.skinJoints[v * 4 + k];
					if (wt <= 0.0f || j < 0 || j >= d.numJoints) continue;
					sk.mWeights[w++] = SBSS::SkinWeight((JPH::uint32)j, wt);
				}
				if (w == 0) continue;
				sk.NormalizeWeights();
				ss->mSkinnedConstraints.push_back(sk);
			}
			ss->CalculateSkinnedConstraintNormals();
		}
		ss->Optimize();

		JPH::SoftBodyCreationSettings cs(ss,
			JPH::RVec3(d.pos[0], d.pos[1], d.pos[2]),
			JPH::Quat::sIdentity(), d.localSpace ? ObjLayers::SOFT_LOCAL : ObjLayers::SOFT);
		cs.mNumIterations  = d.iterations > 0 ? (JPH::uint32)d.iterations : 5;
		cs.mLinearDamping  = d.linearDamping;
		cs.mFriction       = d.friction;
		cs.mPressure       = d.pressure;
		cs.mGravityFactor  = d.gravityFactor;
		cs.mVertexRadius   = d.vertexRadius;
		cs.mUpdatePosition = false;    // stable origin: world = local + pos, forever
		cs.mAllowSleeping  = false;    // pinned verts move by direct writes a sleeper ignores

		JPH::BodyInterface& bi = m_system->GetBodyInterface();
		JPH::Body* body = bi.CreateSoftBody(cs);
		if (!body) { cout << "[NukePhysicsJolt]\tcreateSoftBody FAILED (body pool full?)" << endl; return 0; }
		bi.AddBody(body->GetID(), JPH::EActivation::Activate);
		m_bodies.insert(body->GetID().GetIndexAndSequenceNumber());   // reset() sweeps it too
		return body->GetID().GetIndexAndSequenceNumber();
	}

	void destroySoftBody(uint64_t sb) override { destroyBody(sb); }

	void setSoftBodyVertices(uint64_t sb, const int* idx, const float* worldPos, int n) override
	{
		if (!m_system || !sb || !idx || !worldPos || n <= 0) return;
		JPH::BodyLockWrite lock(m_system->GetBodyLockInterface(), JPH::BodyID((JPH::uint32)sb));
		if (!lock.Succeeded()) return;
		JPH::Body& b = lock.GetBody();
		if (!b.IsSoftBody()) return;
		auto* mp = static_cast<JPH::SoftBodyMotionProperties*>(b.GetMotionProperties());
		const JPH::RMat44 inv = b.GetCenterOfMassTransform().Inversed();
		JPH::Array<JPH::SoftBodyVertex>& vs = mp->GetVertices();
		for (int i = 0; i < n; ++i)
		{
			const int v = idx[i];
			if (v < 0 || v >= (int)vs.size()) continue;
			const JPH::Vec3 lp = JPH::Vec3(inv * JPH::RVec3(worldPos[i * 3 + 0],
			                                                worldPos[i * 3 + 1],
			                                                worldPos[i * 3 + 2]));
			// Pins (invMass 0) are kinematic: their velocity is the drive, kill it. A FREE
			// vertex here is a position CORRECTION (capsule push-out, max-distance leash) -
			// keep its velocity or the sim freezes.
			if (vs[v].mInvMass == 0.0f) vs[v].mVelocity = JPH::Vec3::sZero();
			vs[v].mPosition = lp;
		}
	}

	bool getSoftBodyVertices(uint64_t sb, float* out, int maxVerts) override
	{
		if (!m_system || !sb || !out || maxVerts <= 0) return false;
		JPH::BodyLockRead lock(m_system->GetBodyLockInterface(), JPH::BodyID((JPH::uint32)sb));
		if (!lock.Succeeded()) return false;
		const JPH::Body& b = lock.GetBody();
		if (!b.IsSoftBody()) return false;
		const auto* mp = static_cast<const JPH::SoftBodyMotionProperties*>(b.GetMotionProperties());
		const JPH::RMat44 com = b.GetCenterOfMassTransform();
		const JPH::Array<JPH::SoftBodyVertex>& vs = mp->GetVertices();
		const int n = std::min(maxVerts, (int)vs.size());
		for (int i = 0; i < n; ++i)
		{
			const JPH::RVec3 w = com * vs[i].mPosition;
			out[i * 3 + 0] = (float)w.GetX();
			out[i * 3 + 1] = (float)w.GetY();
			out[i * 3 + 2] = (float)w.GetZ();
		}
		return true;
	}

	void addSoftBodyVelocity(uint64_t sb, const float dv[3]) override
	{
		if (!m_system || !sb || !dv) return;
		JPH::BodyLockWrite lock(m_system->GetBodyLockInterface(), JPH::BodyID((JPH::uint32)sb));
		if (!lock.Succeeded()) return;
		JPH::Body& b = lock.GetBody();
		if (!b.IsSoftBody()) return;
		auto* mp = static_cast<JPH::SoftBodyMotionProperties*>(b.GetMotionProperties());
		const JPH::Vec3 d(dv[0], dv[1], dv[2]);
		for (JPH::SoftBodyVertex& v : mp->GetVertices())
			if (v.mInvMass > 0.0f) v.mVelocity += d;
	}

	void setSoftBodyJoints(uint64_t sb, const float* joints16, int numJoints, bool hardSkin) override
	{
		if (!m_system || !sb || !joints16 || numJoints <= 0 || !m_tempAllocator) return;
		JPH::BodyLockWrite lock(m_system->GetBodyLockInterface(), JPH::BodyID((JPH::uint32)sb));
		if (!lock.Succeeded()) return;
		JPH::Body& b = lock.GetBody();
		if (!b.IsSoftBody()) return;
		auto* mp = static_cast<JPH::SoftBodyMotionProperties*>(b.GetMotionProperties());
		JPH::Array<JPH::Mat44> mats;
		mats.reserve(numJoints);
		for (int j = 0; j < numJoints; ++j)
			mats.push_back(JPH::Mat44::sLoadFloat4x4((const JPH::Float4*)(joints16 + j * 16)));
		mp->SkinVertices(b.GetCenterOfMassTransform(), mats.data(), (JPH::uint)numJoints,
		                 hardSkin, *m_tempAllocator);
	}

	// ---- serialized shapes: appended at the iPhysics vtable END (ABI) ----

	bool cookMeshShape(const float* verts, int vertCount, void** outBlob, int* outSize) override
	{
		if (!verts || vertCount < 3 || !outBlob || !outSize) return false;
		JPH::TriangleList tris;
		const int triCount = vertCount / 3;
		tris.reserve(triCount);
		for (int t = 0; t < triCount; ++t)
		{
			const float* v = verts + t * 9;
			tris.push_back(JPH::Triangle(JPH::Float3(v[0], v[1], v[2]),
			                             JPH::Float3(v[3], v[4], v[5]),
			                             JPH::Float3(v[6], v[7], v[8])));
		}
		JPH::MeshShapeSettings ms(tris);
		JPH::Shape::ShapeResult r = ms.Create();
		if (r.HasError())
		{
			cout << "[NukePhysicsJolt]\tcookMeshShape failed: " << r.GetError() << endl;
			return false;
		}
		std::stringstream ss(std::ios::in | std::ios::out | std::ios::binary);
		JPH::StreamOutWrapper out(ss);
		r.Get()->SaveBinaryState(out);
		const std::string s = ss.str();
		if (s.empty()) return false;
		char* blob = (char*)std::malloc(s.size());
		if (!blob) return false;
		std::memcpy(blob, s.data(), s.size());
		*outBlob = blob;
		*outSize = (int)s.size();
		return true;
	}

	void freeCookedBlob(void* blob) override { std::free(blob); }

	uint64_t createBodyFromCooked(const void* blob, int size, const float pos[3],
	                              const float quat[4], float friction, float restitution) override
	{
		if (!m_system || !blob || size <= 0) return 0;
		std::stringstream ss(std::string((const char*)blob, (size_t)size),
		                     std::ios::in | std::ios::binary);
		JPH::StreamInWrapper in(ss);
		JPH::Shape::ShapeResult r = JPH::Shape::sRestoreFromBinaryState(in);
		if (r.HasError())
		{
			// Stale blob (Jolt upgrade / backend change): the caller re-cooks from source.
			cout << "[NukePhysicsJolt]\tcooked shape restore failed: " << r.GetError() << endl;
			return 0;
		}
		JPH::BodyCreationSettings bcs(r.Get(),
			JPH::RVec3(pos[0], pos[1], pos[2]),
			JPH::Quat(quat[0], quat[1], quat[2], quat[3]).Normalized(),
			JPH::EMotionType::Static, ObjLayers::NON_MOVING);
		bcs.mFriction    = friction;
		bcs.mRestitution = restitution;
		JPH::BodyInterface& bi = m_system->GetBodyInterface();
		JPH::Body* body = bi.CreateBody(bcs);
		if (!body) { cout << "[NukePhysicsJolt]\tcreateBodyFromCooked FAILED (body pool full?)" << endl; return 0; }
		bi.AddBody(body->GetID(), JPH::EActivation::Activate);
		m_bodies.insert(body->GetID().GetIndexAndSequenceNumber());
		return body->GetID().GetIndexAndSequenceNumber();
	}

	void activateBodies(const float mn[3], const float mx[3]) override
	{
		if (!m_system) return;
		const JPH::AABox box(JPH::Vec3(mn[0], mn[1], mn[2]), JPH::Vec3(mx[0], mx[1], mx[2]));
		JPH::BroadPhaseLayerFilter bp;
		JPH::ObjectLayerFilter ol;
		m_system->GetBodyInterface().ActivateBodiesInAABox(box, bp, ol);
	}

	void step(float dt) override
	{
		if (!m_system) return;
		// Characters step BEFORE PhysicsSystem::Update so their pushes land the same step.
		// A CharacterVirtual is not tracked by the system: ExtendedUpdate is the whole move.
		for (auto& kv : m_characters)
		{
			CharRec& c = kv.second;
			c.character->SetLinearVelocity(c.desiredVel);
			JPH::CharacterVirtual::ExtendedUpdateSettings us;
			const JPH::Vec3 up = c.character->GetUp();
			us.mWalkStairsStepUp     = up * c.stepHeight;
			us.mStickToFloorStepDown = -up * c.stickDistance;
			c.character->ExtendedUpdate(dt, m_system->GetGravity(), us,
				m_system->GetDefaultBroadPhaseLayerFilter(ObjLayers::MOVING),
				m_system->GetDefaultLayerFilter(ObjLayers::MOVING),
				{}, {}, *m_tempAllocator);
			c.character->UpdateGroundVelocity();   // AFTER the move: next step reads current motion
		}
		TimeScalePre();   // local time (TimeVolume): scaled bodies enter the solve at v s
		// NUKE_PHYS_PERF=1: per-scene average solver cost, printed once a second of steps.
		static const bool perf = std::getenv("NUKE_PHYS_PERF") != nullptr;
		if (perf)
		{
			const auto t0 = std::chrono::steady_clock::now();
			m_system->Update(dt, 1, m_tempAllocator.get(), m_jobSystem.get());
			const double ms = std::chrono::duration<double, std::milli>(
				std::chrono::steady_clock::now() - t0).count();
			m_perfAcc += ms;
			if (++m_perfN >= 60)
			{
				printf("[JoltPerf]\tscene %p step avg %.2f ms (%u bodies)\n",
				       (void*)this, m_perfAcc / m_perfN, m_system->GetNumActiveBodies(JPH::EBodyType::SoftBody));
				m_perfAcc = 0.0; m_perfN = 0;
			}
		}
		else
			m_system->Update(dt, 1, m_tempAllocator.get(), m_jobSystem.get());
		TimeScalePost();   // ...and leave it at their real velocity
	}

	// ---- characters: appended at the iPhysics vtable END (ABI) ----

	uint64_t createCharacter(const NukeCharacterDesc& d) override
	{
		if (!m_system) return 0;
		// Capsule with the PIVOT AT THE FEET: lifted by halfHeight+radius so the character's
		// position is where it stands.
		const float lift = d.halfHeight + d.radius;
		JPH::RefConst<JPH::Shape> capsule = new JPH::CapsuleShape(d.halfHeight, d.radius);
		JPH::ShapeSettings::ShapeResult shape = JPH::RotatedTranslatedShapeSettings(
			JPH::Vec3(0, lift, 0), JPH::Quat::sIdentity(), capsule).Create();
		if (shape.HasError())
		{
			cout << "[NukePhysicsJolt]\tcharacter shape failed: " << shape.GetError() << endl;
			return 0;
		}
		JPH::Ref<JPH::CharacterVirtualSettings> cs = new JPH::CharacterVirtualSettings();
		cs->mShape            = shape.Get();
		cs->mUp               = JPH::Vec3(d.up[0], d.up[1], d.up[2]).NormalizedOr(JPH::Vec3::sAxisY());
		cs->mMaxSlopeAngle    = JPH::DegreesToRadians(d.maxSlopeDeg);
		cs->mMass             = d.mass;
		cs->mMaxStrength      = d.maxStrength;
		cs->mCharacterPadding = d.padding;
		// Only the bottom sphere supports: otherwise a wall edge touching the side counts as ground.
		cs->mSupportingVolume = JPH::Plane(cs->mUp, -d.radius);
		if (d.innerBody)
		{
			// A kinematic capsule body rides the character's pose, so queries/contacts see it.
			cs->mInnerBodyShape = shape.Get();
			cs->mInnerBodyLayer = ObjLayers::MOVING;
		}
		CharRec rec;
		rec.character = new JPH::CharacterVirtual(cs, JPH::RVec3(d.pos[0], d.pos[1], d.pos[2]),
		                                          JPH::Quat::sIdentity(), 0, m_system.get());
		rec.character->SetCharacterVsCharacterCollision(&m_charVsChar);
		m_charVsChar.Add(rec.character);
		rec.stepHeight    = d.stepHeight;
		rec.stickDistance = d.stickDistance;
		const uint64_t id = m_nextCharId++;
		m_characters[id] = rec;
		return id;
	}

	void destroyCharacter(uint64_t ch) override
	{
		auto it = m_characters.find(ch);
		if (it == m_characters.end()) return;
		m_charVsChar.Remove(it->second.character);
		m_characters.erase(it);   // Ref release destroys the character + its inner body
	}

	void setCharacterVelocity(uint64_t ch, const float v[3]) override
	{
		auto it = m_characters.find(ch);
		if (it != m_characters.end()) it->second.desiredVel = JPH::Vec3(v[0], v[1], v[2]);
	}

	void getCharacterVelocity(uint64_t ch, float v[3]) override
	{
		v[0] = v[1] = v[2] = 0;
		auto it = m_characters.find(ch);
		if (it == m_characters.end()) return;
		const JPH::Vec3 lv = it->second.character->GetLinearVelocity();
		v[0] = lv.GetX(); v[1] = lv.GetY(); v[2] = lv.GetZ();
	}

	void setCharacterPosition(uint64_t ch, const float pos[3]) override
	{
		auto it = m_characters.find(ch);
		if (it != m_characters.end())
			it->second.character->SetPosition(JPH::RVec3(pos[0], pos[1], pos[2]));
	}

	bool getCharacterState(uint64_t ch, float pos[3], int& groundState,
	                       float groundNormal[3], float groundVel[3], uint64_t& groundBody) override
	{
		auto it = m_characters.find(ch);
		if (it == m_characters.end()) return false;
		JPH::CharacterVirtual* c = it->second.character;
		const JPH::RVec3 p = c->GetPosition();
		pos[0] = (float)p.GetX(); pos[1] = (float)p.GetY(); pos[2] = (float)p.GetZ();
		groundState = (int)c->GetGroundState();   // enum orders match (On/Steep/Unsupported/Air)
		const JPH::Vec3 n = c->GetGroundNormal();
		groundNormal[0] = n.GetX(); groundNormal[1] = n.GetY(); groundNormal[2] = n.GetZ();
		const JPH::Vec3 gv = c->GetGroundVelocity();
		groundVel[0] = gv.GetX(); groundVel[1] = gv.GetY(); groundVel[2] = gv.GetZ();
		groundBody = c->GetGroundBodyID().GetIndexAndSequenceNumber();
		return true;
	}

	void setCharacterParams(uint64_t ch, float maxSlopeDeg, float stepHeight, float stickDistance) override
	{
		auto it = m_characters.find(ch);
		if (it == m_characters.end()) return;
		it->second.character->SetMaxSlopeAngle(JPH::DegreesToRadians(maxSlopeDeg));
		it->second.stepHeight    = stepHeight;
		it->second.stickDistance = stickDistance;
	}

	uint64_t characterBodyId(uint64_t ch) override
	{
		auto it = m_characters.find(ch);
		if (it == m_characters.end()) return 0;
		const JPH::BodyID inner = it->second.character->GetInnerBodyID();
		return inner.IsInvalid() ? 0 : inner.GetIndexAndSequenceNumber();
	}

	bool raycastIgnore(const float from[3], const float dir[3], float maxDist, uint64_t ignoreBody,
	                   uint64_t& hitBody, float hitPoint[3], float hitNormal[3]) override
	{
		if (!m_system) return false;
		JPH::Vec3 d(dir[0], dir[1], dir[2]);
		if (d.LengthSq() < 1e-12f || maxDist <= 0.0f) return false;
		d = d.Normalized() * maxDist;
		JPH::RRayCast ray{ JPH::RVec3(from[0], from[1], from[2]), d };
		JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;
		JPH::IgnoreSingleBodyFilter ignore{ JPH::BodyID((JPH::uint32)ignoreBody) };
		m_system->GetNarrowPhaseQuery().CastRay(ray, JPH::RayCastSettings{}, collector,
		                                        {}, {}, ignore);
		if (!collector.HadHit()) return false;
		const JPH::RayCastResult& hit = collector.mHit;
		hitBody = hit.mBodyID.GetIndexAndSequenceNumber();
		JPH::RVec3 p = ray.GetPointOnRay(hit.mFraction);
		hitPoint[0] = (float)p.GetX(); hitPoint[1] = (float)p.GetY(); hitPoint[2] = (float)p.GetZ();
		hitNormal[0] = hitNormal[1] = hitNormal[2] = 0.0f;
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
		return shapeCastIgnore(s, from, quat, dir, maxDist, 0, hitBody, hitPoint, hitNormal);
	}

	bool shapeCastIgnore(const NukeShapeDesc& s, const float from[3], const float quat[4],
	                     const float dir[3], float maxDist, uint64_t ignoreBody,
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
		JPH::IgnoreSingleBodyFilter ignore{ JPH::BodyID((JPH::uint32)ignoreBody) };
		m_system->GetNarrowPhaseQuery().CastShape(cast, settings, JPH::RVec3::sZero(), collector,
		                                          {}, {}, ignore);
		if (!collector.HadHit()) return false;

		const JPH::ShapeCastResult& hit = collector.mHit;
		hitBody = hit.mBodyID2.GetIndexAndSequenceNumber();
		hitPoint[0] = (float)hit.mContactPointOn2.GetX();
		hitPoint[1] = (float)hit.mContactPointOn2.GetY();
		hitPoint[2] = (float)hit.mContactPointOn2.GetZ();
		// mPenetrationAxis points from the cast shape INTO the hit body; the normal is its negation.
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
		// One empty step after the sweep: the broad-phase QuadTree frees the removed bodies'
		// nodes LAZILY on the next update — destroying the system right away trips its
		// free-list destructor assert (first seen with soft bodies).
		if (m_system) step(1.0f / 60.0f);
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
	std::unique_ptr<NukeJobSystem>             m_jobSystem;   // solver jobs on nuke::Jobs
	double m_perfAcc = 0.0;   // NUKE_PHYS_PERF accumulators
	int    m_perfN = 0;
	std::unique_ptr<JPH::PhysicsSystem>        m_system;
	std::unordered_set<JPH::uint32>            m_bodies;      // live handles (reset/validation)
	// Local time (setBodyTimeScale): per-body factor + the velocities parked while frozen.
	struct TimeScaleRec { float s = 1.0f; JPH::Vec3 v = JPH::Vec3::sZero(), w = JPH::Vec3::sZero(); std::vector<JPH::Vec3> vv; /* soft: per vertex */ };
	std::unordered_map<JPH::uint32, TimeScaleRec> m_timeScale;
	// Vehicles' authored forces (setBodyTimeScale on the chassis rescales from these).
	struct VehicleBase
	{
		struct WheelBase { JPH::WheelSettings* ws; float frequency, brake, handBrake; };   // WV or TV settings by type
		int   type = 0;             // 0 wheeled, 1 tracked
		float maxTorque = 0.0f;
		float trackBrake = 0.0f;
		std::vector<WheelBase> wheels;
	};
	std::map<uint64_t, VehicleBase> m_vehBase;

	// Hover vehicles (type 2): stepped by one step listener, no constraint.
	struct HoverRec
	{
		struct Thruster { JPH::Vec3 local; float height, frequency, damping; float length = 0.0f; bool contact = false; };
		JPH::uint32 body = 0;
		std::vector<Thruster> thrusters;
		float thrust = 0, turn = 0, grip = 0, upright = 0, angDamp = 0, brakeForce = 0;
		float inF = 0, inR = 0, inB = 0;
		float s = 1.0f;             // local time scale of the chassis
	};
	std::map<uint64_t, HoverRec> m_hovers;
	struct HoverStepper final : public JPH::PhysicsStepListener
	{
		JoltPhysics* self = nullptr;
		bool added = false;
		void OnStep(const JPH::PhysicsStepListenerContext& ctx) override { if (self) self->StepHovers(ctx); }
	};
	HoverStepper m_hoverStepper;

	// Virtual-capsule character controllers, stepped in step().
	struct CharRec
	{
		JPH::Ref<JPH::CharacterVirtual> character;
		JPH::Vec3 desiredVel = JPH::Vec3::sZero();
		float stepHeight = 0.35f, stickDistance = 0.5f;
	};
	std::map<uint64_t, CharRec>                m_characters;
	uint64_t                                   m_nextCharId = 1;
	JPH::CharacterVsCharacterCollisionSimple   m_charVsChar;  // characters collide with each other
	std::map<uint64_t, JPH::SwingTwistConstraint*> m_joints;  // ragdoll joints (Ref held by the system)
	// Generic constraints (createConstraint): the Jolt constraint + its NukeConstraintDesc type.
	std::map<uint64_t, std::pair<JPH::TwoBodyConstraint*, int>> m_constraints;
	// Wheeled vehicles: the constraint doubles as the step listener; the Ref keeps it alive.
	std::map<uint64_t, JPH::Ref<JPH::VehicleConstraint>> m_vehicles;
	uint64_t                                   m_nextJoint = 1;
};
static JoltPhysics gPhysics;

// ---- plugin export ----
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

	void OnLoad() override {}          // Collider/Rigidbody are ENGINE components
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
