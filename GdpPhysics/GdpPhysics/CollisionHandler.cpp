#include "CollisionHandler.h"
#include "RigidBody.h"
#include <iostream>

namespace gdpPhysics
{
	CollisionHandler::CollisionHandler() {}
	CollisionHandler::~CollisionHandler() {}

	glm::vec3 ProjectOn(glm::vec3 b, glm::vec3 a)
	{
		float mult = ((a.x * b.x) + (a.y * b.y) + (a.z * b.z)) / ((b.x * b.x) + (b.y * b.y) + (b.z * b.z));
		return glm::vec3(b.x * mult, b.y * mult, b.z * mult);
	}

	// Page 224 Chapter 5 Basic Primitive Tests
	// Christer Ericson - Real-time Collision Detection
	bool TestMovingSphereSphere(
		const glm::vec3& s0Center, const float s0Radius,
		const glm::vec3& s1Center, const float s1Radius,
		const glm::vec3& v0, const glm::vec3& v1, float& t)
	{
		// Textbook orange... Eric one
		glm::vec3 s = s1Center - s0Center;	// Vector between sphere centers
		glm::vec3 v = v1 - v0;				// Relative motion of s1 with respect to stationary s0
		float r = s1Radius + s0Radius;		// SUm of sphere radii
		float c = glm::dot(s, s) - r * r;

		if (c < 0.0f) {
			// Already intersecting, not moving towards a collision
			t = 0.0f;
			return true;
		}

		float a = glm::dot(v, v);
		if (a < std::numeric_limits<float>::epsilon())
			return false;		// Spheres not moving relative to eachother

		float b = glm::dot(v, s);
		if (b >= 0.0f)
			return false;		// Spherse not moving towards eachother

		float d = (b * b) - (a * c);
		if (d < 0.0f)
			return false;		// No real-valued root,  spheres do not intersect

		t = (-b - sqrt(d)) / a;
		return true;
	}

	void ClosestPtPointSphere(const glm::vec3& ptA, const glm::vec3& ptB, const float& radiusA, const float& radiusB,
			glm::vec3& clstA, glm::vec3& clstB)		// const means can't change it in here
	{
		glm::vec3 sphereAToB = ptB - ptA;			// vector from sphere A pointing to sphere B
		sphereAToB = glm::normalize(sphereAToB);	// normalize it
		sphereAToB *= radiusA;						// multiply by the radius to get the point on the surface of the sphere
		//glm::vec3 worldPoint = ptA + sphereAToB;	// don't need this since the applyForceAtPoint seems to use relative point (at least that's what it says)
		clstA = sphereAToB;

		// and do it again for the other sphere
		glm::vec3 sphereBToA = ptA - ptB;
		sphereBToA = glm::normalize(sphereBToA);
		sphereBToA *= radiusB;

		clstB = sphereBToA;

		return;
	}

	bool CollisionHandler::CollideSphereSphere(
		float dt, RigidBody* bodyA, SphereShape* sphereA,
		RigidBody* bodyB, SphereShape* sphereB)
	{
		if (bodyA->IsStatic() && bodyB->IsStatic())
			return false;

		glm::vec3 vA = bodyA->m_Position - bodyA->m_PreviousPosition;
		glm::vec3 vB = bodyB->m_Position - bodyB->m_PreviousPosition;

		float t = 0.f;
		if (!TestMovingSphereSphere(bodyA->m_PreviousPosition, sphereA->GetRadius()
			, bodyB->m_PreviousPosition, sphereB->GetRadius()
			, vA, vB, t))
		{
			// No current or future collision
			return false;
		}

		if (t > dt)
			return false; // No collision this update step.

		float fractDt = t / (glm::length(vA) + glm::length(vB));
		fractDt = glm::clamp(fractDt, 0.f, 1.f);
		float revDt = (1.0f - fractDt) * dt;
		bodyA->VerletStep1(-revDt);
		bodyB->VerletStep1(-revDt);

		float totalMass = bodyA->m_Mass + bodyB->m_Mass;

		float aFactor = bodyB->m_Mass / totalMass;
		float bFactor = bodyA->m_Mass / totalMass;

		glm::vec3 ab = bodyB->m_Position - bodyA->m_Position;
		float lengthAB = glm::length(ab);
		float overlap = lengthAB - (sphereA->GetRadius() + sphereB->GetRadius());

		if (overlap < std::numeric_limits<float>::epsilon())
		{
			glm::vec3 overlapVec = ab;
			overlapVec = glm::normalize(overlapVec);
			overlapVec *= -overlap;

			if (!bodyA->m_IsStatic) bodyB->m_Position += overlapVec * bFactor;
			if (!bodyB->m_IsStatic) bodyA->m_Position -= overlapVec * aFactor;

			ab = bodyB->m_Position - bodyA->m_Position;
			lengthAB = glm::length(ab);
		}
		ab /= lengthAB;

		// p = m*v
		// p: momentum
		// m: mass
		// v: velocity

		// Reactions to collisions conserve momentum, and involve both elastic
		// and inelastic components. It "looks right" - meaning I should see
		// what I would expect to happen in the real world: small spheres
		// being pushed around and bouncing off of larger ones, while larger
		// spheres more easily push past smaller ones

		glm::vec3 momentumA = bodyA->m_LinearVelocity * bodyA->m_Mass;
		glm::vec3 momentumB = bodyB->m_LinearVelocity * bodyB->m_Mass;
		glm::vec3 totalMomentum = momentumA + momentumB;

		momentumA = totalMomentum * aFactor;
		momentumB = totalMomentum * bFactor;

		float elasticity = 0.4f;	// This can be changed from a value between 0.0 and 1.0;

		glm::vec3 elasticMomentumB = ab * (glm::length(momentumB) * elasticity) * (-1.0f);
		glm::vec3 inelasticMomentumB = ab * glm::length(momentumB) * (1.0f - elasticity);

		glm::vec3 elasticMomentumA = ab * (glm::length(momentumA) * elasticity);
		glm::vec3 inelasticMomentumA = ab * glm::length(momentumA) * (1.0f - elasticity);

		// And let's apply rotational force to the sphere 
		glm::vec3 closestPtBodyA, closestPtBodyB;
		glm::vec3 posA, posB;
		bodyA->GetPosition(posA);	bodyB->GetPosition(posB);
		ClosestPtPointSphere(posA, posB, sphereA->GetRadius(), sphereB->GetRadius(),
			closestPtBodyA, closestPtBodyB);

		glm::vec3 impactComponent = ProjectOn(bodyB->m_LinearVelocity, bodyA->m_LinearVelocity);
		glm::vec3 impactTangentA = bodyA->m_LinearVelocity - impactComponent;
		glm::vec3 impactTangentB = bodyB->m_LinearVelocity - impactComponent;

		//glm::vec3 angularDirectionA = glm::cross(impactTangentA, bodyA->m_AngularVelocity);
		//glm::vec3 angularDirectionB = glm::cross(impactTangentB, bodyB->m_AngularVelocity);

		// this is something we did in class but I think this is basically what we're supposed to do
		// for the rotational forces on the spheres
		if (glm::dot(impactTangentA, impactTangentB) >= 0)
		{
			bodyA->ApplyImpulseAtPoint(-2.0f * impactComponent * bodyA->m_Mass, closestPtBodyA);
			bodyB->ApplyImpulseAtPoint(2.0f * impactComponent * bodyB->m_Mass, closestPtBodyB);
			//std::cout << "Applying rotational impulse on collision!" << std::endl;
		}
		else
		{
			//sphere->ApplyForce(impactTangent * -1.f * sphere->m_Mass * plane->m_Friction);
			//float multiplier = surfaceVelocity * -1.0f * sphere->m_Mass * plane->m_Friction;
			//glm::vec3 force = sphere->m_AngularVelocity * multiplier;
			//sphere->m_AngularVelocity *= plane->m_Friction;
			//sphere->ApplyForceAtPoint(angularDirection, relativePoint);
			bodyA->m_LinearVelocity -= (elasticMomentumA + inelasticMomentumA) * (bodyA->m_InvMass * bodyA->m_Restitution);
			bodyB->m_LinearVelocity += (elasticMomentumB + inelasticMomentumB) * (bodyB->m_InvMass * bodyB->m_Restitution);

		}

		bodyA->UpdateAcceleration();
		bodyB->UpdateAcceleration();

		//bodyA->m_LinearVelocity -= (elasticMomentumA + inelasticMomentumA) * (bodyA->m_InvMass * bodyA->m_Restitution);
		//bodyB->m_LinearVelocity += (elasticMomentumB + inelasticMomentumB) * (bodyB->m_InvMass * bodyB->m_Restitution);

		//printf("BodyA: Velocity(%.2f, %.2f)", bodyA->m_Velocity.x, bodyA->m_Velocity.y);
		//printf("BodyB: Velocity(%.2f, %.2f)", bodyB->m_Velocity.x, bodyB->m_Velocity.y);

		
		// Friction
		//float numerator = abs(1.0f * planeShape->GetNormal().y);	// normals of the two planes
		float numerator = abs(bodyA->m_LinearVelocity.x * bodyB->m_LinearVelocity.x + 
							bodyA->m_LinearVelocity.y * bodyB->m_LinearVelocity.y +
							bodyA->m_LinearVelocity.z * bodyB->m_LinearVelocity.z);	// 
			//std::cout << "Numerator: " << numerator << std::endl;
		float lengthA = glm::sqrt(bodyA->m_LinearVelocity.length());
		//std::cout << "Length A: " << lengthA << std::endl;
		float lengthB = glm::sqrt(bodyB->m_LinearVelocity.length());
		//std::cout << "Length B: " << lengthB << std::endl;
		float denominator = lengthA * lengthB;
		//std::cout << "Denominator: " << denominator << std::endl; 
		float cosO = numerator / denominator;
		//std::cout << "cosO: " << cosO << std::endl;
		float force = bodyA->m_Mass * bodyB->m_Mass * cosO;
		//std::cout << "Force: " << force << std::endl;
		// Then Force of Friction = mew * N
		float forceFriction = bodyA->m_Friction * bodyB->m_Friction * force;
		//std::cout << "Force Friction: " << forceFriction << std::endl;
		//float forceFriction = sphere->m_Friction * plane->m_Friction * force;		// other potential
		// of course mew is determined by the type of surfaces (object and surface) and whether the object is already moving or not
		// but we're going to simplify and either use the float mew I made here or the friction float on the sphere
		// and now we need the direction...
		// friction is always applied in the opposite direction of the velocity
		
		// friction would be different per body, but something dumb I could try is in the same direction as the opposite body
		glm::vec3 bodyBFriction = bodyA->m_LinearVelocity;
		bodyBFriction = glm::normalize(bodyBFriction);
		bodyBFriction *= forceFriction;

		glm::vec3 bodyAFriction = bodyB->m_LinearVelocity;
		bodyAFriction = glm::normalize(bodyAFriction);
		bodyAFriction *= forceFriction;

		bodyA->ApplyForceAtPoint(bodyAFriction, closestPtBodyA);		// I should hopefully notice they slide slower now
		bodyA->UpdateAcceleration();

		bodyB->ApplyForceAtPoint(bodyBFriction, closestPtBodyB);
		bodyB->UpdateAcceleration();

		// Integrate
		bodyA->VerletStep1(revDt);
		bodyB->VerletStep1(revDt);

		return true;
	}

	bool TestMovingSpherePlane(const glm::vec3& a, const glm::vec3& b, float r, const glm::vec3& norm, float dotProduct)
	{
		// Get distance for both a and b form the plane
		float adist = glm::dot(a, norm) - dotProduct;
		float bdist = glm::dot(b, norm) - dotProduct;

		// Intersects if on different sides of plane (distances have different signs)
		if (adist * bdist < 0.0f)
			return true;

		// Interesects if start or end position within radius form plane
		if (abs(adist) <= r || abs(bdist) < r)			// <---- HERE --- REPLACE bdist compare <= with <
			return true;

		// No intersection
		return false;
	}

	glm::vec3 ClosestPtPointPlane(const glm::vec3& pt, const glm::vec3& norm, float dotProduct)
	{
		float t = glm::dot(norm, pt) - dotProduct;
		return pt - t * norm;
	}

	bool  CollisionHandler::CollideSpherePlane(float dt, RigidBody* sphere, SphereShape* sphereShape,
		RigidBody* plane, PlaneShape* planeShape)
	{
		// TestMovingSpherePlane(...) ericson textbook
		if (!TestMovingSpherePlane(sphere->m_PreviousPosition, sphere->m_Position, sphereShape->GetRadius(),
			planeShape->GetNormal(), planeShape->GetDotProduct()))
		{
			return false;
		}
		// https://www.engineeringtoolbox.com/friction-coefficients-d_778.html
		//float mew = 0.8f;	// clean and dry steel
		float mew = 0.2f;	// idk

		glm::vec3 closestPoint = ClosestPtPointPlane(sphere->m_Position, planeShape->GetNormal(), planeShape->GetDotProduct());
		glm::vec3 overlapVec = closestPoint - sphere->m_Position;
		float overlapLength = glm::length(overlapVec);

		float velocityLength = glm::length(sphere->m_LinearVelocity);

		if (velocityLength > 0.000001f)
		{
			float overlapLength = glm::length(overlapVec);
			float velocity = glm::length(sphere->m_LinearVelocity);

			// fractDt is the overlap "ratio" of the timestep
			float fractDt = sphereShape->GetRadius() * ((sphereShape->GetRadius() / overlapLength) - 1.0f) / velocity;

			// partialDt is the part of the dt contributed to the overlap
			float partialDt = (1.f - fractDt) * dt;

			// Backing up the sphere so it should be just touching the plane
			sphere->VerletStep1(-partialDt);

			// Bouncing off
			glm::vec3 reflect = glm::reflect(sphere->m_LinearVelocity, planeShape->GetNormal());
			sphere->m_LinearVelocity = reflect;

			glm::vec3 previousVelocity = sphere->m_LinearVelocity;
			sphere->m_LinearVelocity = reflect;

			glm::vec3 impactComponent = ProjectOn(sphere->m_LinearVelocity, planeShape->GetNormal());
			glm::vec3 impactTangent = previousVelocity - impactComponent;

			glm::vec3 relativePoint = glm::normalize(closestPoint - sphere->m_Position) * sphereShape->GetRadius();
			float surfaceVelocity = sphereShape->GetRadius() * glm::length(sphere->m_AngularVelocity);

			glm::vec3 angularDirection = glm::cross(impactTangent, sphere->m_AngularVelocity);

			// this is something we did in class but I think this is basically what we're supposed to do
			// for the rotational forces on the spheres
			if (glm::dot(impactTangent, planeShape->GetNormal()) >= 0)
			{
				sphere->ApplyImpulseAtPoint(-2.0f * impactComponent * sphere->m_Mass, relativePoint);
			}
			else
			{
				//sphere->ApplyForce(impactTangent * -1.f * sphere->m_Mass * plane->m_Friction);
				//float multiplier = surfaceVelocity * -1.0f * sphere->m_Mass * plane->m_Friction;
				//glm::vec3 force = sphere->m_AngularVelocity * multiplier;
				sphere->m_AngularVelocity *= plane->m_Friction;
				sphere->ApplyForceAtPoint(angularDirection, relativePoint);
			}

			sphere->UpdateAcceleration();
			//sphere->ApplyForce(impactTangent * -1.f * sphere->m_Mass * plane->m_Friction);

			sphere->VerletStep1(partialDt);

			// Maybe do friction here?
			// Normal force of surface (N) = mass * gravity * cosO (where O is the angle the surface is inclined at)
			// I'm going to use the plane normal to calculate this angle
			// actually I want the angle between the plane and the XZ-plane
			// XZ-plane has y = 0.0f
			
			//float numerator = abs(1.0f * planeShape->GetNormal().x + 0.0f + 1.0f * planeShape->GetNormal().z);
			float numerator = abs(1.0f * planeShape->GetNormal().y);	// normals of the two planes
			//std::cout << "Numerator: " << numerator << std::endl;
			float lengthA = glm::sqrt(1.0f + 0.0f + 1.0f);
			//std::cout << "Length A: " << lengthA << std::endl;
			float lengthB = glm::sqrt(planeShape->GetNormal().x * planeShape->GetNormal().x
									+ planeShape->GetNormal().y * planeShape->GetNormal().y
									+ planeShape->GetNormal().z * planeShape->GetNormal().z);
			//std::cout << "Length B: " << lengthB << std::endl;
			float denominator = lengthA * lengthB;
			//std::cout << "Denominator: " << denominator << std::endl; 
			float cosO = numerator / denominator;
			//std::cout << "cosO: " << cosO << std::endl;
			float force = sphere->m_Mass * sphere->m_Gravity.length() * cosO;
			//std::cout << "Force: " << force << std::endl;
			// Then Force of Friction = mew * N
			float forceFriction = mew * force;
			//std::cout << "Force Friction: " << forceFriction << std::endl;
			//float forceFriction = sphere->m_Friction * plane->m_Friction * force;		// other potential
			// of course mew is determined by the type of surfaces (object and surface) and whether the object is already moving or not
			// but we're going to simplify and either use the float mew I made here or the friction float on the sphere
			// and now we need the direction...
			// friction is always applied in the opposite direction of the velocity
			glm::vec3 revVelo = sphere->m_LinearVelocity;
			revVelo = glm::normalize(revVelo);
			revVelo *= forceFriction * -1.0f;			// ey voila, le friction!

			//std::cout << "Velocity x " << sphere->m_LinearVelocity.x
			//	<< " y " << sphere->m_LinearVelocity.y
			//	<< " z " << sphere->m_LinearVelocity.z
			//	<< "\n revVelo x " << revVelo.x
			//	<< " y " << revVelo.y
			//	<< " z " << revVelo.z << "\n" << std::endl;
			
			sphere->ApplyForceAtPoint(revVelo, relativePoint);		// I should hopefully notice they slide slower now
			//sphere->ApplyForce(revVelo);		
			sphere->UpdateAcceleration();

			sphere->VerletStep1(partialDt);

			closestPoint = ClosestPtPointPlane(sphere->m_Position, planeShape->GetNormal(), planeShape->GetDotProduct());
			overlapVec = closestPoint - sphere->m_Position;
			overlapLength = glm::length(overlapVec);
			if (overlapLength < sphereShape->GetRadius())
			{
				sphere->m_Position += planeShape->GetNormal() * (sphereShape->GetRadius() - overlapLength);
				float velDotNorm = glm::dot(sphere->m_LinearVelocity, planeShape->GetNormal());
				if (velDotNorm < 0.f)
				{
					sphere->m_LinearVelocity -= planeShape->GetNormal() * velDotNorm;
				}
			}
			else
			{
				// Should reduce the velocity after impact
				sphere->m_LinearVelocity *= sphere->m_Restitution;
			}


		}
		/*else
		{
			return false;
		}*/

		return true;
	}

	void CollisionHandler::Collide(
		float dt, std::vector<RigidBody*>& bodies, std::vector<CollidingBodies>& collidingBodies)
	{
		iShape* shapeA;
		iShape* shapeB;

		int bodyCount = bodies.size();
		for (int idxA = 0; idxA < bodyCount - 1; idxA++)
		{
			for (int idxB = idxA + 1; idxB < bodyCount; idxB++)
			{
				bool collision = false;
				
				RigidBody* bodyA = bodies[idxA];
				RigidBody* bodyB = bodies[idxB];

				shapeA = bodyA->GetShape();
				shapeB = bodyB->GetShape();

				if (shapeA->GetShapeType() == eShapeType::ShapeTypeSphere)
				{
					if (shapeB->GetShapeType() == eShapeType::ShapeTypeSphere)
					{
						if (CollideSphereSphere(dt, 
							bodyA, dynamic_cast<SphereShape*>(shapeA), 
							bodyB, dynamic_cast<SphereShape*>(shapeB)))
						{
							collision = true;
						}
					}
					else if (shapeB->GetShapeType() == eShapeType::ShapeTypePlane)
					{
						if (CollideSpherePlane(dt,
							bodyA, SphereShape::Cast(shapeA),
							bodyB, PlaneShape::Cast(shapeB)))
						{
							collision = true;
						}
					}
				}
				else if (shapeA->GetShapeType() == eShapeType::ShapeTypePlane)
				{
					if (shapeB->GetShapeType() == eShapeType::ShapeTypeSphere)
					{
						if (CollideSpherePlane(dt,
							bodyB, SphereShape::Cast(shapeB),
							bodyA, PlaneShape::Cast(shapeA)))
						{
							collision = true;
						}
					}
					else if (shapeB->GetShapeType() == eShapeType::ShapeTypePlane)
					{
						// Nope!
					}
				}

				if (collision)
				{
					// Add collision to vector
					collidingBodies.push_back(CollidingBodies(bodyA, bodyB));
				}
			}
		}
	}
}