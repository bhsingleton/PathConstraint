#ifndef _PathConstraint
#define _PathConstraint
//
// File: PathConstraint.h
//
// Dependency Graph Node: PathConstraint
//
// Author: Ben Singleton
//

#include <maya/MPxConstraint.h>
#include <maya/MTypeId.h> 
#include <maya/MObject.h>
#include <maya/MObjectArray.h>
#include <maya/MObjectHandle.h>
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MDagPath.h>
#include <maya/MDataBlock.h>
#include <maya/MDistance.h>
#include <maya/MAngle.h>
#include <maya/MQuaternion.h>
#include <maya/MFloatArray.h>
#include <maya/MMatrix.h>
#include <maya/MMatrixArray.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MDagMessage.h>
#include <maya/MDagModifier.h>
#include <maya/MCallbackIdArray.h>

#include <iostream>
#include <map>
#include <cmath>

 
class PathConstraint : public MPxConstraint
{

public:

							PathConstraint();
	virtual					~PathConstraint(); 

	virtual MStatus			compute(const MPlug& plug, MDataBlock& data);

	static  void*			creator();
	static  MStatus			initialize();
	virtual	void			postConstructor();
	virtual	long			hashCode();

	virtual	MStatus			legalConnection(const MPlug& plug, const MPlug& otherPlug, bool asSrc, bool& isLegal);
	virtual	MStatus			connectionMade(const MPlug& plug, const MPlug& otherPlug, bool asSrc);
	virtual	MStatus			connectionBroken(const MPlug& plug, const MPlug& otherPlug, bool asSrc);

	static	MStatus			connectPlugs(MPlug& source, MPlug& destination);
	static	MStatus			disconnectPlugs(MPlug& source, MPlug& destination);
	static	MStatus			breakConnections(MPlug& plug, bool source, bool destination);

	const	MObject			targetAttribute() const override;
	const	MObject			weightAttribute() const override;
	const	MObject			constraintRotateOrderAttribute() const override;

	virtual	MStatus			updateConstraintParentInverseMatrix();

	static	MDagPath		getAPathTo(MObject& dependNode);
	static	MObject			getParentOf(MObject& dependNode);

	static	MObject			createMatrixData(MMatrix matrix);
	static	MMatrix			getMatrixData(MObject& matrixData);

	static	MMatrix			createPositionMatrix(MVector position);
	static	MStatus			createRotationMatrix(int forwardAxis, MAngle angle, MMatrix& matrx);

	virtual	MStatus			getUpVector(MDataBlock& data, double parameter, MObject& curve, MVector& upVector);
	static	MStatus			getForwardVector(MObject& curve, double parameter, MVector& forwardVector);
	static	MVector			getSceneUpVector();
	virtual	MVector			getObjectUpVector(MMatrix worldUpMatrix);
	static	MVector			getObjectRotationUpVector(MMatrix worldUpMatrix, MVector worldUpVector);
	static	MStatus			getCurvePoint(MObject& curve, double parameter, MPoint& point);
	static	MStatus			getCurveNormal(MObject& curve, double parameter, MVector& upVector);
	static	MStatus			getParamFromFraction(MObject& curve, double fraction, double& parameter);

	static	MStatus			composeMatrix(int forwardAxis, MVector forwardVector, int upAxis, MVector upVector, MPoint position, MMatrix &matrix);
	static	MMatrix			composeMatrix(MVector x, MVector y, MVector z, MPoint p);

	static	MMatrix			blendMatrices(MMatrix startMatrix, MMatrix endMatrix, float weight);
	static	MMatrix			blendMatrices(MMatrix restMatrix, MMatrixArray matrices, MFloatArray weights);

	static	float			sum(MFloatArray items);

	static	MQuaternion		slerp(MQuaternion startQuat, MQuaternion endQuat, float weight);
	static	MFloatArray		clamp(MFloatArray items);
	static	double			clamp(double value, double min, double max);

public:

	static	MObject		restTranslate;
	static	MObject		restTranslateX;
	static	MObject		restTranslateY;
	static	MObject		restTranslateZ;
	static	MObject		restRotate;
	static	MObject		restRotateX;
	static	MObject		restRotateY;
	static	MObject		restRotateZ;

	static	MObject		offsetTranslate;
	static	MObject		offsetTranslateX;
	static	MObject		offsetTranslateY;
	static	MObject		offsetTranslateZ;
	static	MObject		offsetRotate;
	static	MObject		offsetRotateX;
	static	MObject		offsetRotateY;
	static	MObject		offsetRotateZ;

	static  MObject		uValue;
	static  MObject		fractionMode;
	static  MObject		forwardAxis;
	static	MObject		forwardTwist;
	static  MObject		upAxis;
	static  MObject		worldUpType;
	static  MObject		worldUpVector;
	static  MObject		worldUpVectorX;
	static  MObject		worldUpVectorY;
	static  MObject		worldUpVectorZ;
	static  MObject		worldUpMatrix;

	static  MObject		target;
	static  MObject		targetWeight;
	static  MObject		targetCurve;

	static	MObject		constraintTranslate;
	static	MObject		constraintTranslateX;
	static	MObject		constraintTranslateY;
	static	MObject		constraintTranslateZ;

	static	MObject		constraintRotate;
	static	MObject		constraintRotateX;
	static	MObject		constraintRotateY;
	static	MObject		constraintRotateZ;
	static	MObject		constraintRotateOrder;

	static	MObject		constraintMatrix;
	static	MObject		constraintInverseMatrix;
	static	MObject		constraintWorldMatrix;
	static	MObject		constraintWorldInverseMatrix;
	static	MObject		constraintParentInverseMatrix;
	static	MObject		constraintObject;

public:

	static	MTypeId			id;
	static	MString			outputCategory;
	static	MString			targetCategory;

			MObjectHandle	constraintHandle;
	static	MCallbackId		childAddedCallbackId;

	static	std::map<long, PathConstraint*>	instances;

};

#endif