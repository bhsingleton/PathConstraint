#ifndef _PathConstraint
#define _PathConstraint
//
// File: PathConstraint.h
//
// Dependency Graph Node: pathConstraint
//
// Author: Benjamin H. Singleton
//

#include <maya/MPxConstraint.h>
#include <maya/MObject.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MPlug.h>
#include <maya/MDistance.h>
#include <maya/MAngle.h>
#include <maya/MQuaternion.h>
#include <maya/MEulerRotation.h>
#include <maya/MFloatArray.h>
#include <maya/MVector.h>
#include <maya/MPoint.h>
#include <maya/MMatrix.h>
#include <maya/MMatrixArray.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMessageAttribute.h>
#include <maya/MString.h>
#include <maya/MTypeId.h> 
#include <maya/MGlobal.h>


struct WorldUpSettings
{

	int			worldUpType;
	MVector		worldUpVector;
	MMatrix		worldUpMatrix;

};

 
class PathConstraint : public MPxConstraint
{

public:

							PathConstraint();
	virtual					~PathConstraint(); 

	virtual MStatus			compute(const MPlug& plug, MDataBlock& data);

	static  void*			creator();
	static  MStatus			initialize();

	const	MObject			targetAttribute() const override;
	const	MObject			weightAttribute() const override;
	const	MObject			constraintRotateOrderAttribute() const override;

	static	MMatrix			createPositionMatrix(const MVector& position);
	static	MStatus			createRotationMatrix(const int forwardAxis, const MAngle& angle, MMatrix& matrix);

	static	MStatus			sampleCurveAtParameter(const MObject& curve, const double parameter, const WorldUpSettings& settings, MPoint& position, MVector& forwardVector, MVector& upVector);
	static	MStatus			getForwardVector(const MObject& curve, const double parameter, MVector& forwardVector);
	static	MStatus			getUpVector(const MObject& curve, const double parameter, const WorldUpSettings& settings, const MVector& position, MVector& upVector);
	static	MVector			getObjectRotationUpVector(const MVector& worldUpVector, const MMatrix& worldUpMatrix);
	static	MStatus			getCurvePoint(const MObject& curve, const double parameter, MPoint& point);
	static	MStatus			getCurveNormal(const MObject& curve, const double parameter, MVector& upVector);
	static	MStatus			getParamFromFraction(const MObject& curve, const double fraction, double& parameter);

	static	MStatus			composeMatrix(const int forwardAxis, const MVector& forwardVector, const int upAxis, const MVector& upVector, const MPoint& position, MMatrix &matrix);
	static	MMatrix			composeMatrix(const MVector& xAxis, const MVector& yAxis, const MVector& zAxis, const MPoint& position);

	static	MMatrix			blendMatrices(const MMatrix& startMatrix, const MMatrix& endMatrix, const float weight);
	static	MMatrix			blendMatrices(const MMatrix& restMatrix, const MMatrixArray& matrices, const MFloatArray& weights);

	static	float			sum(const MFloatArray& items);

	static	double			dot(const MQuaternion& quat, const MQuaternion& otherQuat);
	static	MQuaternion		slerp(const MQuaternion& startQuat, const MQuaternion& endQuat, const float weight);
	static	MFloatArray		clamp(const MFloatArray& items);
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

public:

	static	MTypeId			id;
	static	MString			outputCategory;
	static	MString			targetCategory;

};

#endif