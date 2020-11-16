/*
 *  ofxBulletJointBase.h
 *  ofxBullet_v7_joints
 *
 *  Created by Nathanael Inkson November 2020
 *  built upon code by Nick Hardeman.
 *
 */

#pragma once
#include "ofMain.h"
#include "btBulletDynamicsCommon.h"
#include "ofxBulletConstants.h"
#include "ofxBulletUtils.h"
#include "ofxBulletRigidBody.h"

/// Base class for the different joints
class ofxBulletJointBase
{
public:
	virtual ~ofxBulletJointBase();

	virtual btTypedConstraint* createSpecificJoint( btRigidBody* a_shape1, btRigidBody* a_shape2, btTransform const &tr_a, btTransform const &tr_b ) 	= 0;
	virtual btTypedConstraint* createSpecificJoint( btRigidBody* a_shape, btTransform const &tr) 														= 0;
	virtual void _setDefaults() = 0;

	void	create( btDiscreteDynamicsWorld* a_world, ofxBulletRigidBody* a_shape1, ofxBulletRigidBody* a_shape2 );
	void	create( btDiscreteDynamicsWorld* a_world, ofxBulletRigidBody* a_shape, glm::vec3 a_pos );
	
	virtual void add();
	
	btRigidBody* getRigidBodyA() const;
	btRigidBody* getRigidBodyB() const;
	glm::vec3 getPositionA() const;
	glm::vec3 getPositionB() const;

	virtual void draw();

	btTypedConstraint* 			_baseJoint;
	btDiscreteDynamicsWorld*	_world;
protected:
	ofxBulletJointBase();


	glm::vec3					_targetPos;
	// is there two bodies the joint is connecting? if not, what is the target pos //
	bool						_bTwoBodies;
	bool						_bCreated;
	bool						_bAdded;
private:
};

