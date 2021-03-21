/*
 *  ofxBulletConeJoint.cpp
 *  ofxBullet_v7_joints
 *
 *  Created by Nathanael Inkson January 2021
 *  built upon code by Nick Hardeman.
 *
 */

#include "ofxBulletConeJoint.h"

//--------------------------------------------------------------
ofxBulletConeJoint::ofxBulletConeJoint()
{}

//--------------------------------------------------------------
ofxBulletConeJoint::~ofxBulletConeJoint() {
	remove();
}
//---------------------------------------------------------------------------------------------------------------------------
void
ofxBulletConeJoint::createCone(btDiscreteDynamicsWorld* a_world, ofxBulletRigidBody* body1, ofxBulletRigidBody* body2, btTransform const &tr_a, btTransform const &tr_b ) {

	_world = a_world;
	_joint = new btConeTwistConstraint(*body1->getRigidBody(), *body2->getRigidBody(), tr_a, tr_b);

	body1->setActivationState( DISABLE_DEACTIVATION );
	body2->setActivationState( DISABLE_DEACTIVATION );

	_baseJoint = _joint;
	_bTwoBodies = true;
	_bCreated	= true;
	_setDefaults();
}
//--------------------------------------------------------------
btTypedConstraint*
ofxBulletConeJoint::createSpecificJoint(btRigidBody* a_shape1, btRigidBody* a_shape2, btTransform const &tr_a, btTransform const &tr_b ) {
	_joint = new btConeTwistConstraint(*a_shape2, *a_shape1, tr_a, tr_b);
	return _joint;
}

//--------------------------------------------------------------
btTypedConstraint*
ofxBulletConeJoint::createSpecificJoint(btRigidBody* a_shape, btTransform const &tr ) {
	_joint = new btConeTwistConstraint(*a_shape, tr);
	return _joint;
}

//--------------------------------------------------------------
void ofxBulletConeJoint::remove() {
	_world->removeConstraint(_joint);
	delete _joint;
	_joint = nullptr;
}
//--------------------------------------------------------------
void ofxBulletConeJoint::add()
{
	_world->addConstraint(_joint, false);
	_bAdded = true;
}
//--------------------------------------------------------------
void ofxBulletConeJoint::_setDefaults() {
	// set constraint limit
    const btScalar low = -M_PI;
    const btScalar high = M_PI;
    _joint->setLimit( low, high );
}
//--------------------------------------------------------------
void ofxBulletConeJoint::setLimits(float const low, float const high) {
	_joint->setLimit( low, high );
}
//--------------------------------------------------------------

