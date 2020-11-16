/*
 *  ofxBulletJoint.h
 *  ofxBullet_v7_joints
 *
 *  Created by Nick Hardeman on 7/6/11.
 *
 */

#pragma once
#include "ofxBulletJointBase.h"

// creates a btGeneric6DofConstraint joint, free rotation, no constraints //
class ofxBulletJoint : public ofxBulletJointBase
{
public:
	ofxBulletJoint();
	~ofxBulletJoint();

	btTypedConstraint* createSpecificJoint(btRigidBody* a_shape1, btRigidBody* a_shape2, btTransform const &tr_a, btTransform const &tr_b ) override;
	btTypedConstraint* createSpecificJoint(btRigidBody* a_shape,  btTransform const &tr ) override;
	
	/******************************/
	// call before calling add() //
	void	setLinearLowerLimit( glm::vec3 a_limit );
	void	setLinearLowerLimit( float a_x, float a_y, float a_z );
	void	setLinearUpperLimit( glm::vec3 a_limit );
	void	setLinearUpperLimit( float a_x, float a_y, float a_z );
	void	setAngularLowerLimit( glm::vec3 a_limit );
	void	setAngularLowerLimit( float a_x, float a_y, float a_z );
	void	setAngularUpperLimit( glm::vec3 a_limit );
	void	setAngularUpperLimit( float a_x, float a_y, float a_z );
	/******************************/
	
	glm::vec3 getPivotAWorldPos();
	glm::vec3 getPivotBWorldPos();
	
	void	updatePivotPos( const glm::vec3 a_pos, float a_length );
	
	void	drawJointConstraints();
	
	void	remove();
	
protected:
	void _setDefaults() override;
	
private:
	btGeneric6DofConstraint*	_joint;
};

