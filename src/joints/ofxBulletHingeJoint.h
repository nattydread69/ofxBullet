/*
 *  ofxBulletHingeJoint.h
 *  ofxBullet_v7_joints
 *
 *  Created by Nathanael Inkson November 2020
 *  built upon code by Nick Hardeman.

 *
 */
#pragma once
#include "ofxBulletJointBase.h"

/// Creates a btHingeConstraint joint
class ofxBulletHingeJoint : public ofxBulletJointBase
{
public:
	ofxBulletHingeJoint();
	~ofxBulletHingeJoint();
	void createHinge(btDiscreteDynamicsWorld* a_world, ofxBulletRigidBody* body1, ofxBulletRigidBody* body2, btTransform const &tr_a, btTransform const &tr_b);
	void createHinge(btDiscreteDynamicsWorld* a_world, ofxBulletRigidBody* body1, ofxBulletRigidBody* body2,
	                 const glm::vec3& pivotInA, const glm::vec3& pivotInB,
	                 const glm::vec3& axisInA, const glm::vec3& axisInB, bool useReferenceFrameA = false);
	
	void add() override;
	void remove();
	void draw() override;
	void setLimits(float const low, float const high);
protected:
	btTypedConstraint* createSpecificJoint(btRigidBody* a_shape1, btRigidBody* a_shape2, btTransform const &tr_a, btTransform const &tr_b ) override;
	btTypedConstraint* createSpecificJoint(btRigidBody* a_shape, btTransform const &tr ) override;
	void _setDefaults() override;
	
private:
	btHingeConstraint*	_joint;
	glm::vec3 _pivotA;
	glm::vec3 _pivotB;
	glm::vec3 _pivotPoint;
	bool _hasPivots;
};

