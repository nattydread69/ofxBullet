/*
 *  ofxBulletHingeJoint.cpp
 *  ofxBullet_v7_joints
 *
 *  Created by Nathanael Inkson November 2020
 *  built upon code by Nick Hardeman.
 *
 */

#include "ofxBulletHingeJoint.h"

//--------------------------------------------------------------
ofxBulletHingeJoint::ofxBulletHingeJoint() : _hasPivots(false)
{}

//--------------------------------------------------------------
ofxBulletHingeJoint::~ofxBulletHingeJoint() {
	remove();
}

//---------------------------------------------------------------------------------------------------------------------------
void
ofxBulletHingeJoint::
createHinge(btDiscreteDynamicsWorld* a_world, ofxBulletRigidBody* body1, ofxBulletRigidBody* body2, const glm::vec3& pivotInA, const glm::vec3& pivotInB,
	 	 	const glm::vec3& axisInA, const glm::vec3& axisInB, bool useReferenceFrameA )
{
	btVector3 pA(pivotInA.x, pivotInA.y, pivotInA.z);
	btVector3 pB(pivotInB.x, pivotInB.y, pivotInB.z);
	btVector3 axA(axisInA.x, axisInA.y, axisInA.z);
	btVector3 axB(axisInB.x, axisInB.y, axisInB.z);

	_world = a_world;
	_joint = new btHingeConstraint(*body1->getRigidBody(), *body2->getRigidBody(), pA, pB, axA, axB, useReferenceFrameA);

	// _joint->setAngularOnly(true); // this has the effect of disconnecting the hinge

	_baseJoint = _joint;

	body1->getRigidBody()->setActivationState( DISABLE_DEACTIVATION );
	body2->getRigidBody()->setActivationState( DISABLE_DEACTIVATION );

	_pivotA = pivotInA;
	_pivotB = pivotInB;
	_bTwoBodies = true;
	_bCreated	= true;
	_hasPivots = true;
	_pivotPoint = (getPositionA() + getPositionB())/2.0f;

	_setDefaults();
}
//---------------------------------------------------------------------------------------------------------------------------
void
ofxBulletHingeJoint::createHinge(btDiscreteDynamicsWorld* a_world, ofxBulletRigidBody* body1, ofxBulletRigidBody* body2, btTransform const &tr_a, btTransform const &tr_b ) {

	_world = a_world;
	_joint = new btHingeConstraint(*body1->getRigidBody(), *body2->getRigidBody(), tr_a, tr_b, true);

	body1->setActivationState( DISABLE_DEACTIVATION );
	body2->setActivationState( DISABLE_DEACTIVATION );

	_baseJoint = _joint;
	_bTwoBodies = true;
	_bCreated	= true;
	_setDefaults();
}
//--------------------------------------------------------------
btTypedConstraint*
ofxBulletHingeJoint::createSpecificJoint(btRigidBody* a_shape1, btRigidBody* a_shape2, btTransform const &tr_a, btTransform const &tr_b ) {
	_joint = new btHingeConstraint(*a_shape1, *a_shape2, tr_a, tr_b, true);
	return _joint;
}
//--------------------------------------------------------------
btTypedConstraint*
ofxBulletHingeJoint::createSpecificJoint(btRigidBody* a_shape, btTransform const &tr ) {
	_joint = new btHingeConstraint(*a_shape, tr, false);
	return _joint;
}

//--------------------------------------------------------------
void ofxBulletHingeJoint::remove() {
	_world->removeConstraint(_joint);
	delete _joint;
	_joint = nullptr;
}
//--------------------------------------------------------------
void ofxBulletHingeJoint::add()
{
	_world->addConstraint(_joint, false);
	_bAdded = true;
}
//--------------------------------------------------------------
void ofxBulletHingeJoint::_setDefaults() {
	// set constraint limit
    const btScalar low = -M_PI;
    const btScalar high = M_PI;
    _joint->setLimit( low, high );
}
//--------------------------------------------------------------
void ofxBulletHingeJoint::setLimits(float const low, float const high) {
	_joint->setLimit( low, high );
}

//--------------------------------------------------------------
void ofxBulletHingeJoint::draw() {
	if(!_bCreated) {ofLog(OF_LOG_ERROR, "ofxBulletHingeJoint :: draw : must call create() first"); return;}

	if (!_hasPivots)
	{
		ofxBulletJointBase::draw();
	}
	else
	{
		glm::vec3 pa;
		glm::vec3 pb = getPositionB();
		if(_bTwoBodies) {
			pa = getPositionA();
		} else {
			pa = _targetPos;
		}

		ofDrawLine(pa , _pivotPoint );
		ofDrawLine(pb , _pivotPoint );
	}
}
//--------------------------------------------------------------




