/*
 *  ofxBulletJointBase.cpp
 *  ofxBullet_v7_joints
 *
 *  Created by Nathanael Inkson November 2020
 *  built upon code by Nick Hardeman..
 *
 */

#include "ofxBulletJointBase.h"

//--------------------------------------------------------------
ofxBulletJointBase::ofxBulletJointBase() {
	_bCreated		= false;
	_bAdded			= false;
	_bTwoBodies		= false;
	_targetPos = glm::vec3(0, 0, 0);
}

//--------------------------------------------------------------
ofxBulletJointBase::~ofxBulletJointBase() {
	_baseJoint = nullptr;
}

//--------------------------------------------------------------
void ofxBulletJointBase::create( btDiscreteDynamicsWorld* a_world, ofxBulletRigidBody* a_shape1, ofxBulletRigidBody* a_shape2 ) {
	_world = a_world;
	// we should have these always influenced by the joint, so don't let them go to sleep //
	a_shape1->setActivationState( DISABLE_DEACTIVATION );
	a_shape2->setActivationState( DISABLE_DEACTIVATION );
	
	glm::vec3 diff = a_shape2->getPosition() - a_shape1->getPosition();
    glm::quat tquat = a_shape1->getRotationQuat();
//    diff = diff * glm::inverse( tquat );
    diff = glm::inverse( tquat ) * diff;

	btTransform frameInA = btTransform::getIdentity();
	frameInA.setOrigin( btVector3(btScalar(-diff.x), btScalar(-diff.y), btScalar(-diff.z)) );
	btTransform frameInB = btTransform::getIdentity();
	frameInB.setOrigin( btVector3(btScalar(0.), btScalar(0.), btScalar(0.)) );

	_baseJoint = createSpecificJoint(a_shape2->getRigidBody(), a_shape1->getRigidBody(), frameInA, frameInB);
	
	_setDefaults();
	
	_bTwoBodies	= true;
	_bCreated	= true;
}

//--------------------------------------------------------------
void ofxBulletJointBase::create( btDiscreteDynamicsWorld* a_world, ofxBulletRigidBody* a_shape, glm::vec3 a_pos ) {
	_world = a_world;
	// we should have these always influenced by the joint, so don't let them go to sleep //
	a_shape->setActivationState( DISABLE_DEACTIVATION );
	
	btVector3 localPivot	= a_shape->getRigidBody()->getCenterOfMassTransform().inverse() * btVector3(a_pos.x, a_pos.y, a_pos.z);
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin( localPivot );

	_baseJoint = createSpecificJoint(a_shape->getRigidBody(), tr);
	
	_setDefaults();
	
	_targetPos = glm::vec3(a_pos.x, a_pos.y, a_pos.z);
	_bTwoBodies = false;
	_bCreated	= true;
}
/******************************************************/

//--------------------------------------------------------------
void ofxBulletJointBase::add() {
	_world->addConstraint(_baseJoint, true);
	_bAdded = true;
}

//--------------------------------------------------------------
btRigidBody* ofxBulletJointBase::getRigidBodyA() const {
	return (btRigidBody*)&_baseJoint->getRigidBodyA();
}

//--------------------------------------------------------------
btRigidBody* ofxBulletJointBase::getRigidBodyB() const {
	return (btRigidBody*)&_baseJoint->getRigidBodyB();
}

//--------------------------------------------------------------
glm::vec3 ofxBulletJointBase::getPositionA() const {
	return ofGetVec3fPosFromRigidBody( getRigidBodyA() );
}

//--------------------------------------------------------------
glm::vec3 ofxBulletJointBase::getPositionB() const {
	return ofGetVec3fPosFromRigidBody( getRigidBodyB() );
}

//--------------------------------------------------------------
void ofxBulletJointBase::draw() {
	if(!_bCreated) {ofLog(OF_LOG_ERROR, "ofxBulletJointBase :: draw : must call create() first"); return;}
	
	glm::vec3 pa;
	glm::vec3 pb;

	pb = getPositionB();
	if(_bTwoBodies) {
		pa = getPositionA();
	} else {
		pa = _targetPos;
	}
    
    ofDrawLine( pa, pb );
}

