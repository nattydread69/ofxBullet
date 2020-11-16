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

void
ofxBulletHingeJoint::
createHinge(btDiscreteDynamicsWorld* a_world, btRigidBody *rbA, btRigidBody *rbB, const btVector3& pivotInA, const btVector3& pivotInB,
	 	 	const btVector3& axisInA, const btVector3& axisInB, bool useReferenceFrameA )
{
	std::cout << "ofxBulletHingeJoint::createHinge" << std::endl;
	_world = a_world;
	_joint = new btHingeConstraint(*rbA, *rbB, pivotInA, pivotInB, axisInA, axisInB, useReferenceFrameA);

	// _joint->setAngularOnly(true); // this has the effect of disconnecting the hinge

	_baseJoint = _joint;
	std::cout << "created Hinge" << std::endl;

	rbA->setActivationState( DISABLE_DEACTIVATION );
	rbB->setActivationState( DISABLE_DEACTIVATION );

	_bTwoBodies = true;
	_bCreated	= true;
	_pivotA = glm::vec3( pivotInA.getX(), pivotInA.getY(), pivotInA.getZ());
	_pivotB = glm::vec3( pivotInB.getX(), pivotInB.getY(), pivotInB.getZ());
	_hasPivots = true;
	_pivotPoint = (getPositionA() + getPositionB())/2.0f;

	_setDefaults();
	std::cout << "END ofxBulletHingeJoint::createHinge" << std::endl;
}
//--------------------------------------------------------------
btTypedConstraint*
ofxBulletHingeJoint::createSpecificJoint(btRigidBody* a_shape1, btRigidBody* a_shape2, btTransform const &tr_a, btTransform const &tr_b ) {
	_joint = new btHingeConstraint(*a_shape2, *a_shape1, tr_a, tr_b, true);
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
	cout << "ofxBulletHingeJoint :: remove : " << endl;
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




