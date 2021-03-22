/*
 *  ofxBulletConeJoint.h
 *  ofxBullet_v7_joints
 *
 *  Created by Nathanael Inkson November 2020
 *  built upon code by Nick Hardeman.

 *
 */
#pragma once
#include "ofxBulletJointBase.h"

/// Creates a btConeTwistConstraint joint
class ofxBulletConeJoint : public ofxBulletJointBase
{
public:
	ofxBulletConeJoint();
	~ofxBulletConeJoint();

	void createCone(btDiscreteDynamicsWorld* a_world, ofxBulletRigidBody* body1, ofxBulletRigidBody* body2, btTransform const &tr_a, btTransform const &tr_b );

	btTypedConstraint* createSpecificJoint(btRigidBody* a_shape1, btRigidBody* a_shape2, btTransform const &tr_a, btTransform const &tr_b ) override;
	btTypedConstraint* createSpecificJoint(btRigidBody* a_shape, btTransform const &tr ) override;
	
	void add() override;
	void remove();
	void setLimits(float const swing1, float const swing2, float const twist);
protected:
	void _setDefaults() override;
	
private:
	btConeTwistConstraint* _joint;
};

