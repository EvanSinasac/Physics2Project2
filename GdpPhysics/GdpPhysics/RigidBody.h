#pragma once

#include <gdpPhysics/interfaces/iRigidBody.h>
#include <gdpPhysics/interfaces/cRigidBodyDesc.h>
#include <gdpPhysics/interfaces/iShape.h>

#include "CollisionHandler.h"

namespace gdpPhysics
{
	class RigidBody : public iRigidBody
	{
	public:
		RigidBody(const cRigidBodyDesc& desc, iShape* shape);
		~RigidBody();

		iShape* GetShape();

		bool IsStatic() const;

		virtual void GetPosition(glm::vec3& positionOut);
		virtual void SetPosition(const glm::vec3& positionIn);

		virtual void GetOrientation(glm::quat& orientationOut);
		virtual void SetOrientation(const glm::quat& orientationIn);

		virtual void ApplyForce(const glm::vec3& force);
		virtual void ApplyForceAtPoint(const glm::vec3& force, const glm::vec3& relativePoint);

		virtual void ApplyImpulse(const glm::vec3& impulse);
		virtual void ApplyImpulseAtPoint(const glm::vec3& impulse, const glm::vec3& relativePoint);

		virtual void ApplyTorque(const glm::vec3& torque);
		virtual void ApplyTorqueImpulse(const glm::vec3& torqueImpulse);

		void SetGravityAcceleration(const glm::vec3& gravity);
		void UpdateAcceleration();

		void KillForces();
		void ApplyDamping(float dt);
		void VerletStep1(float dt);
		void VerletStep2(float dt);
		void VerletStep3(float dt);

		friend class CollisionHandler;
	private:
		bool m_IsStatic;

		float m_Mass;
		float m_InvMass;

		glm::vec3 m_LinearAcceleration;
		glm::vec3 m_AngularAcceleration;

		glm::vec3 m_PreviousPosition;

		glm::vec3 m_Position;
		glm::vec3 m_LinearVelocity;
		glm::vec3 m_Force;

		glm::quat m_Rotation;
		glm::vec3 m_AngularVelocity;
		glm::vec3 m_Torque;

		float m_Friction;
		float m_Restitution;

		iShape* m_Shape;

		float m_LinearDamping;
		float m_AngularDamping;

		glm::vec3 m_Gravity;

		RigidBody(const RigidBody&) { }
		RigidBody& operator=(const RigidBody&) { return *this; }
	};
}