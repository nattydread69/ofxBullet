/*
 *  ofxBulletJoint.h
 *  ofxBullet_v7_joints
 *
 *  Created by Nick Hardeman on 7/6/11.
 *  Copyright 2011 Arnold Worldwide. All rights reserved.
 *
 */

#pragma once
#include "ofMain.h"
#include "btBulletDynamicsCommon.h"
#include "ofxBulletConstants.h"
#include "ofxBulletUtils.h"
#include "ofxBulletBaseShape.h"

// creates a btGeneric6DofConstraint joint, free rotation, no constraints //
class ofxBulletJoint {
public:
	ofxBulletJoint();
	~ofxBulletJoint();
	
	void	create( btDiscreteDynamicsWorld* $world, ofxBulletBaseShape* $shape1, ofxBulletBaseShape* $shape2 );
	void	create( btDiscreteDynamicsWorld* $world, ofxBulletBaseShape* $shape, ofVec3f $pos );
	
	/******************************/
	// call before calling add() //
	void	setLinearLowerLimit( ofVec3f $limit );
	void	setLinearLowerLimit( float $x, float $y, float $z );
	void	setLinearUpperLimit( ofVec3f $limit );
	void	setLinearUpperLimit( float $x, float $y, float $z );
	void	setAngularLowerLimit( ofVec3f $limit );
	void	setAngularLowerLimit( float $x, float $y, float $z );
	void	setAngularUpperLimit( ofVec3f $limit );
	void	setAngularUpperLimit( float $x, float $y, float $z );
	/******************************/
	
	void	add();
	
	ofVec3f getPivotAWorldPos();
	ofVec3f getPivotBWorldPos();
	
	void	draw();
	
private:
	btDiscreteDynamicsWorld*	_world;
	btGeneric6DofConstraint*	_joint;
	
	bool						_bCreated;
	bool						_bAdded;
};
