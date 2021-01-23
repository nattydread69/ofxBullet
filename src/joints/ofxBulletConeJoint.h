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

	btTypedConstraint* createSpecificJoint(btRigidBody* a_shape1, btRigidBody* a_shape2, btTransform const &tr_a, btTransform const &tr_b ) override;
	btTypedConstraint* createSpecificJoint(btRigidBody* a_shape, btTransform const &tr ) override;
	
	void add() override;
	void remove();
	void setLimits(float const low, float const high);
protected:
	void _setDefaults() override;
	
private:
	btConeTwistConstraint* _joint;
};

