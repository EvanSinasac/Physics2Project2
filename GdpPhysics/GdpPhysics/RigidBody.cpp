#include "RigidBody.h"

namespace gdpPhysics
{
	RigidBody::RigidBody(const cRigidBodyDesc& desc, iShape* shape)
		: iRigidBody()
		, m_Shape(shape)
		, m_Mass(desc.mass)
		, m_IsStatic(desc.IsStatic)
		, m_Position(desc.position)
		, m_LinearVelocity(desc.linearVelocity)
		, m_Rotation(desc.rotation)
		, m_AngularVelocity(desc.angularDamping)
		, m_LinearDamping(desc.linearDamping)
		, m_AngularDamping(desc.angularDamping)
		, m_Restitution(desc.restitution)
		, m_Friction(desc.friction)
	{
		if (m_IsStatic || desc.mass <= 0.0f)
		{
			m_Mass = 0.f;
			m_InvMass = 0.f;
			m_IsStatic;
		}
		else
		{
			m_Mass = desc.mass;
			m_InvMass = 1.0f / desc.mass;
		}
	}

	RigidBody::~RigidBody() {}

	bool RigidBody::IsStatic() const
	{
		return m_IsStatic;
	}

	iShape* RigidBody::GetShape()
	{
		return m_Shape;
	}

	void RigidBody::GetPosition(glm::vec3& positionOut)
	{
		positionOut = m_Position;
	}

	void RigidBody::SetPosition(const glm::vec3& positionIn)
	{
		m_Position = positionIn;
	}

	void RigidBody::GetOrientation(glm::quat& orientationOut)
	{
		orientationOut = m_Rotation;
	}

	void RigidBody::SetOrientation(const glm::quat& orientationIn)
	{
		m_Rotation = orientationIn;
	}

	void RigidBody::ApplyForce(const glm::vec3& force)
	{
		m_Force += force;
	}

	void RigidBody::ApplyForceAtPoint(const glm::vec3& force, const glm::vec3& relativePoint)
	{
		ApplyForce(force);
		ApplyTorque(glm::cross(relativePoint, force));
	}

	void RigidBody::ApplyImpulse(const glm::vec3& impulse)
	{
		m_LinearVelocity += impulse * m_InvMass;
	}

	void RigidBody::ApplyImpulseAtPoint(const glm::vec3& impulse, const glm::vec3& relativePoint)
	{
		ApplyTorqueImpulse(glm::cross(relativePoint, impulse));
	}

	void RigidBody::ApplyTorque(const glm::vec3& torque)
	{
		m_Torque += torque;
	}

	void RigidBody::ApplyTorqueImpulse(const glm::vec3& torqueImpulse)
	{
		m_AngularVelocity += torqueImpulse;
	}

	void RigidBody::SetGravityAcceleration(const glm::vec3& gravity)	
	{
		m_Gravity = gravity;
	}

	void RigidBody::UpdateAcceleration()								
	{
		if (m_IsStatic)
			return;

		m_LinearAcceleration = m_Force * m_InvMass + m_Gravity;
		m_AngularAcceleration = m_Torque;
	}

	void RigidBody::KillForces()										
	{
		m_Force = glm::vec3(0.0f);
		m_Torque = glm::vec3(0.0f);
	}

	void RigidBody::ApplyDamping(float dt)								
	{
		m_LinearVelocity *= pow(1.f - m_LinearDamping, dt);
		m_AngularVelocity *= pow(1.f - m_AngularDamping, dt);

		if (glm::length(m_LinearVelocity) < 0.001f)
			m_LinearVelocity = glm::vec3(0.0f);
	}

	void RigidBody::VerletStep1(float dt)								
	{
		if (m_IsStatic)
			return;

		m_PreviousPosition = m_Position;
		m_Position += (m_LinearVelocity + m_LinearAcceleration * (dt * 0.5f)) * dt;

		glm::vec3 axis = (m_AngularVelocity + m_AngularAcceleration * (dt * 0.5f)) * dt;
		float angle = glm::length(axis);
		axis = glm::normalize(axis);
		if (angle != 0.f)
		{
			glm::quat rot = glm::angleAxis(angle, axis);
			m_Rotation *= rot;
		}
	}

	void RigidBody::VerletStep2(float dt)								
	{
		if (m_IsStatic)
			return;

		m_LinearVelocity += m_LinearAcceleration * (dt * 0.5f);

		m_AngularVelocity += m_AngularAcceleration * (dt * 0.5f);
	}

	void RigidBody::VerletStep3(float dt)								
	{
		VerletStep2(dt);
	}
}
