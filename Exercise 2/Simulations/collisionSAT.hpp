// header file:
#include <vector>

#include "Collider.h"

using namespace GamePhysics;

// tool data structures/functions called by the collision detection method, you can ignore the details here
namespace collisionTools {
	struct Projection {
		float min, max;
	};

	// Get the pair of edges
	inline std::vector<Vec3> getPairOfEdges(const Collider& obj_A, const Collider& obj_B)
	{
		const Vec3* edges1 = &(obj_A.basisAxis[0]);
		const Vec3* edges2 = &(obj_B.basisAxis[0]);

		std::vector<Vec3> results;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Vec3 vector = cross(edges1[i], edges2[j]);
				if (normNoSqrt(vector) > 0)
					normalize(vector);
					results.push_back(vector);
			}
		}
		return results;
	}

	// project a shape on an axis
	inline Projection project(const Collider& obj, const Vec3& axis)
	{
		// Get corners
		const std::vector<Vec3>& cornersWorld = obj.worldCorners;
		float min = dot(cornersWorld[0], axis);
		float max = min;
		for (int i = 1; i < cornersWorld.size(); i++)
		{
			float p = dot(cornersWorld[i], axis);
			if (p < min) {
				min = p;
			}
			else if (p > max) {
				max = p;
			}
		}
		Projection projection;
		projection.max = max;
		projection.min = min;
		return projection;
	}

	inline bool overlap(Projection p1, Projection p2)
	{
		return !((p1.max > p2.max && p1.min > p2.max) || (p2.max > p1.max && p2.min > p1.max));
	}

	inline float getOverlap(Projection p1, Projection p2)
	{
		return std::min(p1.max, p2.max) - std::max(p1.min, p2.min);
	}

	static inline Vec3 contactPoint(
		const Vec3& pOne,
		const Vec3& dOne,
		float oneSize,
		const Vec3& pTwo,
		const Vec3& dTwo,
		float twoSize,

		// If this is true, and the contact point is outside
		// the edge (in the case of an edge-face contact) then
		// we use one's midpoint, otherwise we use two's.
		bool useOne)
	{
		Vec3 toSt, cOne, cTwo;
		float dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
		float denom, mua, mub;

		smOne = normNoSqrt(dOne);
		smTwo = normNoSqrt(dTwo);
		dpOneTwo = dot(dTwo, dOne);

		toSt = pOne - pTwo;
		dpStaOne = dot(dOne, toSt);
		dpStaTwo = dot(dTwo, toSt);

		denom = smOne * smTwo - dpOneTwo * dpOneTwo;

		// Zero denominator indicates parrallel lines
		if (abs(denom) < 0.0001f) {
			return useOne ? pOne : pTwo;
		}

		mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
		mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

		// If either of the edges has the nearest point out
		// of bounds, then the edges aren't crossed, we have
		// an edge-face contact. Our point is on the edge, which
		// we know from the useOne parameter.
		if (mua > oneSize ||
			mua < -oneSize ||
			mub > twoSize ||
			mub < -twoSize)
		{
			return useOne ? pOne : pTwo;
		}
		else
		{
			cOne = pOne + dOne * mua;
			cTwo = pTwo + dTwo * mub;

			return cOne * 0.5 + cTwo * 0.5;
		}
	}

	inline Vec3 handleVertexToface(const Collider& obj, const Vec3& toCenter)
	{
		const std::vector<Vec3>& cornersWorld = obj.worldCorners;
		float min = 1000;
		Vec3 vertex;
		for (int i = 0; i < cornersWorld.size(); i++)
		{
			float value = dot(cornersWorld[i], toCenter);
			if (value < min)
			{
				vertex = cornersWorld[i];
				min = value;
			}
		}
		return vertex;
	}

	inline ContactInfo checkCollisionSATHelper(const Collider& obj_A, const Collider& obj_B)
	{
		ContactInfo info;
		info.isValid = false;
		Vec3 collisionPoint = Vec3::ZERO;
		float smallOverlap = 10000.0f;
		Vec3 axis;
		int index;
		int fromWhere = -1;
		bool bestSingleAxis = false;

		Vec3 toCenter = obj_B.transform->position - obj_A.transform->position;

		const Vec3* axes1 = &(obj_A.basisAxis[0]);
		const Vec3* axes2 = &(obj_B.basisAxis[0]);
		std::vector<Vec3> axes3 = getPairOfEdges(obj_A, obj_B);
		// loop over the axes1 A axes
		for (int i = 0; i < 3; i++) {
			// project both shapes onto the axis
			Projection p1 = project(obj_A, axes1[i]);
			Projection p2 = project(obj_B, axes1[i]);
			// do the projections overlap?
			if (!overlap(p1, p2)) {
				// then we can guarantee that the shapes do not overlap
				return info;
			}
			else {
				// get the overlap
				float o = getOverlap(p1, p2);
				// check for minimum
				if (o < smallOverlap) {
					// then set this one as the smallest
					smallOverlap = o;
					axis = axes1[i];
					index = i;
					fromWhere = 0;
				}
			}
		}
		// loop over the axes2
		for (int i = 0; i < 3; i++) {
			// project both shapes onto the axis
			Projection p1 = project(obj_A, axes2[i]);
			Projection p2 = project(obj_B, axes2[i]);
			// do the projections overlap?
			if (!overlap(p1, p2)) {
				// then we can guarantee that the shapes do not overlap
				return  info;
			}
			else {
				// get the overlap
				float o = getOverlap(p1, p2);
				// check for minimum
				if (o < smallOverlap) {
					// then set this one as the smallest
					smallOverlap = o;
					axis = axes2[i];
					index = i;
					fromWhere = 1;
					bestSingleAxis = true;
				}
			}
		}
		int whichEdges = 0;
		// loop over the axes3
		for (int i = 0; i < axes3.size(); i++) {
			// project both shapes onto the axis
			Projection p1 = project(obj_A, axes3[i]);
			Projection p2 = project(obj_B, axes3[i]);
			// do the projections overlap?
			if (!overlap(p1, p2)) {
				// then we can guarantee that the shapes do not overlap
				return info;
			}
			else {
				// get the overlap
				float o = getOverlap(p1, p2);
				// check for minimum
				if (o < smallOverlap) {
					// then set this one as the smallest
					smallOverlap = o;
					axis = axes3[i];
					index = i;
					whichEdges = i;
					fromWhere = 2;
				}
			}
		}
		// if we get here then we know that every axis had overlap on it
		// so we can guarantee an intersection
		Vec3 normal;
		switch (fromWhere) {
		case 0: {
			normal = axis;
			if (dot(axis, toCenter) <= 0)
			{
				normal = normal * -1.0f;
			}
			collisionPoint = handleVertexToface(obj_B, normal);
		}break;
		case 1: {
			normal = axis;
			if (dot(axis, toCenter) <= 0)
			{
				normal = normal * -1.0f;
			}
			collisionPoint = handleVertexToface(obj_A, normal * -1);
		}break;
		case 2: {
			normal = axis;
			if (dot(axis, toCenter) <= 0)
			{
				normal = normal * -1.0f;
			}

			Vec3 ptOnOneEdge(0.5, 0.5, 0.5);
			Vec3 ptOnTwoEdge(0.5,0.5,0.5);

			for (int i = 0; i < 3; i++)
			{
				if (i == whichEdges / 3)
					ptOnOneEdge.value[i] = 0;
				else if (dot(axes1[i], normal) < 0) 
					ptOnOneEdge.value[i] = -ptOnOneEdge.value[i];

				if (i == whichEdges % 3) 
					ptOnTwoEdge.value[i] = 0;
				else if (dot(axes2[i], normal) > 0) 
					ptOnTwoEdge.value[i] = -ptOnTwoEdge.value[i];
			}
			ptOnOneEdge = obj_A.transform->transformation.transformVector(ptOnOneEdge);
			ptOnTwoEdge = obj_B.transform->transformation.transformVector(ptOnTwoEdge);

			collisionPoint = contactPoint(ptOnOneEdge,
				axes1[whichEdges / 3],
				obj_A.size.value[(whichEdges / 3)],
				ptOnTwoEdge,
				axes2[whichEdges % 3],
				obj_B.size[(whichEdges % 3)],
				bestSingleAxis);
		}break;
		}

		info.isValid = true;
		info.point = collisionPoint;
		info.depth = smallOverlap;
		info.normal = normal * -1;

		return info;
	}
}

inline ContactInfo checkCollisionSAT(GamePhysics::Collider* obj_A, GamePhysics::Collider* obj_B) {
	return collisionTools::checkCollisionSATHelper(*obj_A, *obj_B);
}
