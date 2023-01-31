#include "Transform.h"

GamePhysics::Transform::Transform()
{
	position = Vec3(0, 0, 0);
	rotation = Quat(0, 0, 0, 1);

	transformation.initId();
	inverseTransformation.initId();
}

void GamePhysics::Transform::update()
{
	Mat4 translation;
	translation.initTranslation(position.x, position.y, position.z);
	transformation = rotation.getRotMat() * translation;
	
	inverseTransformation = transformation.inverse();
}

void GamePhysics::Transform::setTransformation(const Mat4& T)
{
	transformation = T;
	inverseTransformation = transformation.inverse();

	position = Vec3(transformation.value[3][0], transformation.value[3][1], transformation.value[3][2]);
	rotation = Quat(transformation);
}
