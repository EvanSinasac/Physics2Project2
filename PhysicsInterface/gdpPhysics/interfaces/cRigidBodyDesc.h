#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

namespace gdpPhysics
{
	class cRigidBodyDesc
	{
	public:
		cRigidBodyDesc()
			: mass(1.f)
			, IsStatic(false)
			, position(glm::vec3(0.f))
			, linearVelocity(glm::vec3(0.f)) 
			, linearDamping (0.1f)
			, angularDamping (0.1f)
			, restitution(0.8f)
			, friction(0.95f) {}

		float mass;

		bool IsStatic;

		glm::vec3 position;
		glm::vec3 linearVelocity;

		glm::quat rotation;
		glm::vec3 angularVelocity;

		float linearDamping;
		float angularDamping;
		float restitution;
		float friction;
	};
}