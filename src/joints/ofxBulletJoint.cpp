/*
 *  ofxBulletJoint.cpp
 *  ofxBullet_v7_joints
 *
 *  Created by Nick Hardeman on 7/6/11.
 *
 */

#include "ofxBulletJoint.h"

//--------------------------------------------------------------
ofxBulletJoint::ofxBulletJoint() {
}

//--------------------------------------------------------------
ofxBulletJoint::~ofxBulletJoint() {
	remove();
}

//--------------------------------------------------------------
btTypedConstraint*
ofxBulletJoint::createSpecificJoint(btRigidBody* a_shape1, btRigidBody* a_shape2, btTransform const &tr_a, btTransform const &tr_b )
{
	_joint = new btGeneric6DofConstraint(*a_shape2, *a_shape1, tr_a, tr_b, true);
	return _joint;
}

//--------------------------------------------------------------
btTypedConstraint*
ofxBulletJoint::createSpecificJoint( btRigidBody* a_shape, btTransform const &tr ) {

	_joint = new btGeneric6DofConstraint(*a_shape, tr, false);
	return _joint;
}

//--------------------------------------------------------------
void ofxBulletJoint::_setDefaults() {
	for (unsigned i = 0; i < 6; i++)
	{
		_joint->setParam(BT_CONSTRAINT_STOP_CFM,0.8, i);
		_joint->setParam(BT_CONSTRAINT_STOP_ERP,0.1, i);
	}
}

/******************************************************/
// call these before add() and after create //
void ofxBulletJoint::setLinearLowerLimit( glm::vec3 a_limit ) {
	setLinearLowerLimit( a_limit.x, a_limit.y, a_limit.z );
}
void ofxBulletJoint::setLinearLowerLimit( float a_x, float a_y, float a_z ) {
	_joint->setLinearLowerLimit( btVector3(a_x, a_y, a_z) );
}
void ofxBulletJoint::setLinearUpperLimit( glm::vec3 a_limit ) {
	setLinearUpperLimit( a_limit.x, a_limit.y, a_limit.z );
}
void ofxBulletJoint::setLinearUpperLimit( float a_x, float a_y, float a_z ) {
	_joint->setLinearUpperLimit( btVector3(a_x, a_y, a_z) );
}
void ofxBulletJoint::setAngularLowerLimit( glm::vec3 a_limit ) {
	setAngularLowerLimit( a_limit.x, a_limit.y, a_limit.z );
}
void ofxBulletJoint::setAngularLowerLimit( float a_x, float a_y, float a_z ) {
	_joint->setAngularLowerLimit( btVector3(a_x, a_y, a_z) );
}
void ofxBulletJoint::setAngularUpperLimit( glm::vec3 a_limit ) {
	setAngularUpperLimit( a_limit.x, a_limit.y, a_limit.z );
}
void ofxBulletJoint::setAngularUpperLimit( float a_x, float a_y, float a_z ) {
	_joint->setAngularUpperLimit( btVector3(a_x, a_y, a_z) );
}
/******************************************************/

//--------------------------------------------------------------
void ofxBulletJoint::remove() {
	cout << "ofxBulletJoint :: remove : " << endl;
	_world->removeConstraint(_joint);
	delete _joint;
	_joint = NULL;
}

//--------------------------------------------------------------
glm::vec3 ofxBulletJoint::getPivotAWorldPos() {
	btQuaternion rotQuat	= _joint->getCalculatedTransformA().getRotation();
	btVector3 btaxis		= rotQuat.getAxis();
	btVector3 frameA		= _joint->getFrameOffsetA().getOrigin();
	glm::vec3 dir			= glm::vec3(frameA.getX(), frameA.getY(), frameA.getZ());
    dir = glm::normalize(dir);
    glm::quat tq = glm::angleAxis( rotQuat.getAngle(), glm::vec3( btaxis.getX(), btaxis.getY(), btaxis.getZ()) );
    dir = tq * dir;
//    dir.rotateRad(rotQuat.getAngle(), glm::vec3(btaxis.getX(), btaxis.getY(), btaxis.getZ() ));
	dir *= -frameA.length();
	return dir + getPivotBWorldPos();
}

//--------------------------------------------------------------
glm::vec3 ofxBulletJoint::getPivotBWorldPos() {
	btVector3 trb = _joint->getCalculatedTransformB().getOrigin();
    return glm::vec3( trb.getX(), trb.getY(), trb.getZ() );
}

//--------------------------------------------------------------
void ofxBulletJoint::updatePivotPos( const glm::vec3 a_pos, float a_length ) {
	if(!_bCreated) {ofLog(OF_LOG_ERROR, "ofxBulletJoint :: updatePivotPos : must call create() first"); return;}
	
	_joint->getFrameOffsetA().setOrigin( btVector3(a_pos.x, a_pos.y, a_pos.z) );
	
	_targetPos = glm::vec3( a_pos.x, a_pos.y, a_pos.z );
}

//--------------------------------------------------------------
// draws the length of the joint and locations of pivots, but if bodies are sprung more than the length,
// will not connect all the way //
void ofxBulletJoint::drawJointConstraints() {
	glm::vec3 pa = getPivotAWorldPos();
	glm::vec3 pb = getPivotBWorldPos();
	
	ofDrawLine( pa, pb );
	
	ofSetColor(255, 0, 0);
	ofDrawSphere(pa, .5);
	ofSetColor(0, 0, 255);
	ofDrawSphere(pb, .5);
}

