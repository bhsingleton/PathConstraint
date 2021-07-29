//
// File: PathConstraintNode.cpp
//
// Dependency Graph Node: PathConstraint
//
// Author: Ben Singleton
//

#include "PathConstraint.h"


MObject PathConstraint::restTranslate;
MObject PathConstraint::restTranslateX;
MObject PathConstraint::restTranslateY;
MObject PathConstraint::restTranslateZ;
MObject PathConstraint::restRotate;
MObject PathConstraint::restRotateX;
MObject PathConstraint::restRotateY;
MObject PathConstraint::restRotateZ;

MObject PathConstraint::uValue;
MObject PathConstraint::fractionMode;
MObject PathConstraint::forwardAxis;
MObject PathConstraint::forwardTwist;
MObject PathConstraint::upAxis;
MObject PathConstraint::worldUpType;
MObject PathConstraint::worldUpVector;
MObject PathConstraint::worldUpVectorX;
MObject PathConstraint::worldUpVectorY;
MObject PathConstraint::worldUpVectorZ;
MObject PathConstraint::worldUpMatrix;

MObject PathConstraint::target;
MObject PathConstraint::targetWeight;
MObject PathConstraint::targetCurve;

MObject PathConstraint::constraintTranslate;
MObject PathConstraint::constraintTranslateX;
MObject PathConstraint::constraintTranslateY;
MObject PathConstraint::constraintTranslateZ;
MObject PathConstraint::constraintRotate;
MObject PathConstraint::constraintRotateX;
MObject PathConstraint::constraintRotateY;
MObject PathConstraint::constraintRotateZ;
MObject PathConstraint::constraintRotateOrder;
MObject PathConstraint::constraintMatrix;
MObject PathConstraint::constraintInverseMatrix;
MObject PathConstraint::constraintWorldMatrix;
MObject PathConstraint::constraintWorldInverseMatrix;
MObject PathConstraint::constraintParentInverseMatrix;
MObject PathConstraint::constraintObject;

MTypeId PathConstraint::id(0x00131805);
MString	PathConstraint::targetCategory("Target");
MString	PathConstraint::outputCategory("Output");

std::map<long, PathConstraint*> PathConstraint::instances = std::map<long, PathConstraint*>();
MCallbackId	PathConstraint::childAddedCallbackId;


PathConstraint::PathConstraint() {}


PathConstraint::~PathConstraint()
/**
Destructor.
*/
{

	this->instances.erase(this->hashCode());

};


MStatus PathConstraint::compute(const MPlug& plug, MDataBlock& data)
/**
This method should be overridden in user defined nodes.
Recompute the given output based on the nodes inputs.
The plug represents the data value that needs to be recomputed, and the data block holds the storage for all of the node's attributes.
The MDataBlock will provide smart handles for reading and writing this node's attribute values.
Only these values should be used when performing computations!

@param plug: Plug representing the attribute that needs to be recomputed.
@param data: Data block containing storage for the node's attributes.
@return: Return status.
*/
{

	MStatus status;
 
	// Check requested attribute
	//
	MObject attribute = plug.attribute(&status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MFnAttribute fnAttribute(attribute, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	if (fnAttribute.hasCategory(PathConstraint::outputCategory))
	{
		
		// Get input data handles
		//
		MDataHandle restTranslateHandle = data.inputValue(PathConstraint::restTranslate, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle restRotateHandle = data.inputValue(PathConstraint::restRotate, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintParentInverseMatrixHandle = data.inputValue(PathConstraint::constraintParentInverseMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintRotateOrderHandle = data.inputValue(PathConstraint::constraintRotateOrder, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MArrayDataHandle targetArrayHandle = data.inputArrayValue(PathConstraint::target, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle uValueHandle = data.inputValue(PathConstraint::uValue, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle fractionModeHandle = data.inputValue(PathConstraint::fractionMode, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle forwardAxisHandle = data.inputValue(PathConstraint::forwardAxis, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle forwardTwistHandle = data.inputValue(PathConstraint::forwardTwist, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle upAxisHandle = data.inputValue(PathConstraint::upAxis, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Get values from handles
		//
		MMatrix constraintParentInverseMatrix = constraintParentInverseMatrixHandle.asMatrix();

		MVector restTranslate = restTranslateHandle.asVector();
		int constraintRotateOrder = constraintRotateOrderHandle.asInt();
		MEulerRotation restRotate = MEulerRotation(restRotateHandle.asVector(), MEulerRotation::RotationOrder(constraintRotateOrder));

		MMatrix restTranslateMatrix = PathConstraint::createPositionMatrix(restTranslate);
		MMatrix restRotateMatrix = restRotate.asMatrix();

		MMatrix restMatrix = restRotateMatrix * restTranslateMatrix;
		MMatrix restWorldMatrix = restMatrix * constraintParentInverseMatrix.inverse();

		double uValue = uValueHandle.asDouble();
		bool fractionMode = fractionModeHandle.asBool();
		int forwardAxis = forwardAxisHandle.asShort();
		MAngle forwardTwist = forwardTwistHandle.asAngle();
		int upAxis = upAxisHandle.asShort();

		// Create forward twist matrix
		//
		MMatrix forwardTwistMatrix;

		status = PathConstraint::createRotationMatrix(forwardAxis, forwardTwist, forwardTwistMatrix);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Collect target matrices
		//
		unsigned int targetCount = targetArrayHandle.elementCount();

		MFloatArray targetWeights = MFloatArray(targetCount);
		MMatrixArray targetMatrices = MMatrixArray(targetCount);

		MDataHandle targetHandle;
		MDataHandle targetWeightHandle, targetCurveHandle;

		MObject targetCurve;
		double parameter;
		MPoint position;
		MVector forwardVector, upVector;
		MMatrix targetMatrix, targetParentMatrix;

		for (unsigned int i = 0; i < targetCount; i++)
		{

			// Jump to array element
			//
			status = targetArrayHandle.jumpToElement(i);
			CHECK_MSTATUS_AND_RETURN_IT(status)

			targetHandle = targetArrayHandle.inputValue(&status);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			// Get target data handles
			//
			targetWeightHandle = targetHandle.child(PathConstraint::targetWeight);
			targetCurveHandle = targetHandle.child(PathConstraint::targetCurve);

			// Get weight value
			//
			targetWeights[i] = targetWeightHandle.asFloat();

			// Get curve parameter
			//
			targetCurve = targetCurveHandle.asNurbsCurve();
			parameter = uValue;

			if (fractionMode)
			{

				status = PathConstraint::getParamFromFraction(targetCurve, uValue, parameter);
				CHECK_MSTATUS_AND_RETURN_IT(status);
				
			}

			// Get forward/up vectors
			//
			status = PathConstraint::getCurvePoint(targetCurve, parameter, position);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			status = PathConstraint::getForwardVector(targetCurve, parameter, forwardVector);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			status = this->getUpVector(data, parameter, targetCurve, upVector);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			// Compose transform matrix
			//
			status = PathConstraint::composeMatrix(forwardAxis, forwardVector, upAxis, upVector, position, targetMatrix);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			targetMatrices[i] = forwardTwistMatrix * targetMatrix;

		}

		// Get output data handles
		//
		MDataHandle constraintTranslateXHandle = data.outputValue(PathConstraint::constraintTranslateX, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintTranslateYHandle = data.outputValue(PathConstraint::constraintTranslateY, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintTranslateZHandle = data.outputValue(PathConstraint::constraintTranslateZ, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintRotateXHandle = data.outputValue(PathConstraint::constraintRotateX, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintRotateYHandle = data.outputValue(PathConstraint::constraintRotateY, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintRotateZHandle = data.outputValue(PathConstraint::constraintRotateZ, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintMatrixHandle = data.outputValue(PathConstraint::constraintMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintInverseMatrixHandle = data.outputValue(PathConstraint::constraintInverseMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintWorldMatrixHandle = data.outputValue(PathConstraint::constraintWorldMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle constraintWorldInverseMatrixHandle = data.outputValue(PathConstraint::constraintWorldInverseMatrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Compute weighted constraint matrix
		//
		MMatrix constraintWorldMatrix = PathConstraint::blendMatrices(restWorldMatrix, targetMatrices, targetWeights);
		MMatrix constraintMatrix = constraintWorldMatrix * constraintParentInverseMatrix;

		// Set translation constraint
		//
		MTransformationMatrix transformationMatrix = MTransformationMatrix(MMatrix(constraintMatrix));
		transformationMatrix.reorderRotation(MTransformationMatrix::RotationOrder(constraintRotateOrder + 1));

		MVector constraintTranslate = transformationMatrix.getTranslation(MSpace::kTransform, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDistance::Unit internalUnit = MDistance::internalUnit();

		constraintTranslateXHandle.setMDistance(MDistance(constraintTranslate.x, internalUnit));
		constraintTranslateYHandle.setMDistance(MDistance(constraintTranslate.y, internalUnit));
		constraintTranslateZHandle.setMDistance(MDistance(constraintTranslate.z, internalUnit));

		constraintTranslateXHandle.setClean();
		constraintTranslateYHandle.setClean();
		constraintTranslateZHandle.setClean();

		// Set rotation constraint
		//
		MEulerRotation constraintRotate = transformationMatrix.eulerRotation();

		constraintRotateXHandle.setMAngle(MAngle(constraintRotate.x, MAngle::kRadians));
		constraintRotateYHandle.setMAngle(MAngle(constraintRotate.y, MAngle::kRadians));
		constraintRotateZHandle.setMAngle(MAngle(constraintRotate.z, MAngle::kRadians));

		constraintRotateXHandle.setClean();
		constraintRotateYHandle.setClean();
		constraintRotateZHandle.setClean();

		// Commit matrices to handles
		//
		constraintMatrixHandle.setMMatrix(constraintMatrix);
		constraintInverseMatrixHandle.setMMatrix(constraintMatrix.inverse());
		constraintWorldMatrixHandle.setMMatrix(constraintWorldMatrix);
		constraintWorldInverseMatrixHandle.setMMatrix(constraintWorldMatrix.inverse());

		constraintMatrixHandle.setClean();
		constraintInverseMatrixHandle.setClean();
		constraintWorldMatrixHandle.setClean();
		constraintWorldInverseMatrixHandle.setClean();

		// Mark data block as clean
		//
		status = data.setClean(plug);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		return MS::kSuccess;

	}

	return MS::kUnknownParameter;

};


void onChildAdded(MDagPath& child, MDagPath& parent, void* clientData)
/**
Child added callback function.
This function will handle updating the constraint's parent handle.

@param child: The newly added child.
@param parent: The parent owner.
@param clientData: Pointer to any client data passed on creation.
@return: void
*/
{

	MStatus status;

	// Iterate through instances
	//
	std::map<long, PathConstraint*>::iterator iter;

	long hashCode;
	PathConstraint* constraint;

	for (iter = PathConstraint::instances.begin(); iter != PathConstraint::instances.end(); iter++)
	{

		// Check if child is derived from constraint
		//
		hashCode = iter->first;
		constraint = iter->second;

		if (constraint->constraintHandle.object() == child.node())
		{

			status = constraint->updateConstraintParentInverseMatrix();
			CHECK_MSTATUS(status);

		}

	}

};


MStatus PathConstraint::legalConnection(const MPlug& plug, const MPlug& otherPlug, bool asSrc, bool& isLegal)
/**
This method allows you to check for legal connections being made to attributes of this node.
You should return kUnknownParameter to specify that maya should handle this connection if you are unable to determine if it is legal.

@param plug: Attribute on this node.
@param otherPlug: Attribute on other node.
@param asSrc: Is this plug a source of the connection.
@param isLegal: Set this to true if the connection is legal otherwise false.
@return: MStatus
*/
{

	// Check the plug attribute
	//
	MObject attribute = plug.attribute();

	if (attribute == PathConstraint::constraintObject && !asSrc)
	{

		// Verify other node is a dag node
		//
		MObject otherNode = otherPlug.node();
		MObject otherAttribute = otherPlug.attribute();

		if (otherNode.hasFn(MFn::kDagNode) && otherAttribute.hasFn(MFn::kMessageAttribute))
		{

			isLegal = true;

		}
		else
		{

			isLegal = false;

		}

		return MS::kSuccess;

	}

	return MS::kUnknownParameter;

};


MStatus PathConstraint::connectionMade(const MPlug& plug, const MPlug& otherPlug, bool asSrc)
/**
This method gets called when connections are made to attributes of this node.
You should return kUnknownParameter to specify that maya should handle this connection or if you want maya to process the connection as well.

@param plug: Attribute on this node.
@param otherPlug: Attribute on other node.
@param asSrc: Is this plug a source of the connection.
@return: MStatus
*/
{

	MStatus status;

	// Check the plug attribute
	//
	MObject attribute = plug.attribute();

	if (attribute == PathConstraint::constraintObject && !asSrc)
	{

		this->constraintHandle = MObjectHandle(otherPlug.node());

	}

	return MS::kUnknownParameter;

};


MStatus PathConstraint::connectionBroken(const MPlug& plug, const MPlug& otherPlug, bool asSrc)
/**
This method gets called when connections are broken with attributes of this node.
You should return kUnknownParameter to specify that maya should handle this connection or if you want maya to process the connection as well.

@param plug: Attribute on this node.
@param otherPlug: Attribute on other node.
@param asSrc: Is this plug a source of the connection.
@return: MStatus
*/
{

	MStatus status;

	// Check the plug attribute
	//
	MObject attribute = plug.attribute();

	if (attribute == PathConstraint::constraintObject && !asSrc)
	{

		this->constraintHandle = MObjectHandle();

	}

	return MS::kUnknownParameter;

};


MStatus PathConstraint::connectPlugs(MPlug& source, MPlug& destination)
/**
Connects the two supplied plugs.

@param source: The source plug.
@param destination: The destination plug.
@return: Return status;
*/
{

	MStatus status;

	// Check if plugs are valid
	//
	if (source.isNull() || destination.isNull())
	{

		return MS::kFailure;

	}

	// Execute dag modifier
	//
	MDagModifier dagModifier;

	status = dagModifier.connect(source, destination);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return dagModifier.doIt();

};


MStatus PathConstraint::disconnectPlugs(MPlug& source, MPlug& destination)
/**
Disconnects the two supplied plugs.

@param source: The source plug.
@param destination: The destination plug.
@return: Return status;
*/
{

	MStatus status;

	// Check if plugs are valid
	//
	if (source.isNull() || destination.isNull())
	{

		return MS::kFailure;

	}

	// Execute dag modifier
	//
	MDagModifier dagModifier;

	status = dagModifier.disconnect(source, destination);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return dagModifier.doIt();

};


MStatus PathConstraint::breakConnections(MPlug& plug, bool source, bool destination)
/**
Breaks the specified connections from the supplied plug.

@param plug: The plug the break connections from.
@param source: Specifies if the source plug should be broken.
@param destination: Specifies if the destination plugs should be broken.
@return: Return status;
*/
{

	MStatus status;

	// Check if source connections should be broken
	//
	if (source)
	{

		MPlug otherPlug = plug.source();

		if (!otherPlug.isNull())
		{

			status = PathConstraint::disconnectPlugs(otherPlug, plug);
			CHECK_MSTATUS_AND_RETURN_IT(status);

		}

	}
	
	// Check if destination plugs should be broken
	//
	if (destination)
	{

		MPlugArray otherPlugs;

		bool isConnected = plug.destinations(otherPlugs, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		unsigned int numOtherPlugs = otherPlugs.length();

		for (unsigned int i = 0; i < numOtherPlugs; i++)
		{

			status = PathConstraint::disconnectPlugs(plug, otherPlugs[i]);
			CHECK_MSTATUS_AND_RETURN_IT(status);

		}

	}

	return status;

};


MStatus PathConstraint::updateConstraintParentInverseMatrix()
/**
Updates the plug connected to the constraintParentInverseMatrix plug.
This function should be called by the childAdded callback whenever the constraint object's parent is changed.
This will ensure the correct worldMatrix plug is used.

@return: Return status.

*/
{

	MStatus status;

	// Check if constraint handle is still alive
	//
	if (!this->constraintHandle.isAlive())
	{

		return MS::kFailure;

	}

	MObject constraintObject = this->constraintHandle.object();

	// Break connections to plug
	//
	MPlug constraintParentInverseMatrixPlug = MPlug(this->thisMObject(), PathConstraint::constraintParentInverseMatrix);

	status = PathConstraint::breakConnections(constraintParentInverseMatrixPlug, true, false);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Check if constraint object has a parent
	//
	MObject parent = PathConstraint::getParentOf(constraintObject);

	if (!parent.isNull())
	{

		// Initialize function set from dag path
		//
		MDagPath dagPath = PathConstraint::getAPathTo(parent);

		MFnDagNode fnDagNode(dagPath, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Connect parent's worldInverseMatrix plug to constraintParentInverseMatrix plug
		// This will bypass any cyclical dependencies from the offsetParentMatrix plug!
		//
		MPlug worldInverseMatrixPlug = fnDagNode.findPlug("worldInverseMatrix", false, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		status = worldInverseMatrixPlug.selectAncestorLogicalIndex(dagPath.instanceNumber());
		CHECK_MSTATUS_AND_RETURN_IT(status);

		status = PathConstraint::connectPlugs(worldInverseMatrixPlug, constraintParentInverseMatrixPlug);
		CHECK_MSTATUS_AND_RETURN_IT(status);

	}
	else
	{

		// Reset plug matrix
		//
		MObject matrixData = PathConstraint::createMatrixData(MMatrix::identity);

		status = constraintParentInverseMatrixPlug.setMObject(matrixData);
		CHECK_MSTATUS_AND_RETURN_IT(status);

	}

	return status;

};


MObject PathConstraint::createMatrixData(MMatrix matrix)
/**
Converts a matrix into a data object compatible with plugs.

@param matrix: The matrix to convert.
@return: A matrix data object.
*/
{

	MFnMatrixData fnMatrixData;

	MObject matrixData = fnMatrixData.create();
	fnMatrixData.set(matrix);

	return matrixData;

};


MMatrix PathConstraint::getMatrixData(MObject& matrixData)
/**
Converts a data object into a matrix compatible with plugs.

@param matrixData: The data object to convert.
@return: A matrix.
*/
{

	MFnMatrixData fnMatrixData(matrixData);
	return fnMatrixData.matrix();

};


MDagPath PathConstraint::getAPathTo(MObject& dependNode)
/**
Returns the dag path for the supplied dag node.

@param dependNode: The node to get a path to.
@return: The dag path.
*/
{

	// Verify this is a dag node
	//
	if (!dependNode.hasFn(MFn::kDagNode))
	{

		return MDagPath();

	}

	// Get a path to dag node
	//
	MDagPath dagPath;
	MDagPath::getAPathTo(dependNode, dagPath);

	return dagPath;

};


MObject PathConstraint::getParentOf(MObject& dependNode)
/**
Returns the parent for the supplied dag node.

@param dependNode: The node to get the parent for.
@return: The parent node.
*/
{

	// Verify this is a dag node
	//
	if (!dependNode.hasFn(MFn::kDagNode))
	{

		return MObject::kNullObj;

	}

	// Initialize function set
	//
	MDagPath dagPath = PathConstraint::getAPathTo(dependNode);
	MFnDagNode fnDagNode(dagPath);

	unsigned int parentCount = fnDagNode.parentCount();

	if (parentCount == 1)
	{

		return fnDagNode.parent(0);

	}
	else
	{

		return MObject::kNullObj;

	}

};


MMatrix PathConstraint::createPositionMatrix(MVector position)
/**
Creates a position matrix from the given vector.

@param position: The vector to convert.
@return: The new position matrix.
*/
{

	double matrix[4][4] = {
		{ 1.0, 0.0, 0.0, 0.0 },
		{ 0.0, 1.0, 0.0, 0.0 },
		{ 0.0, 0.0, 1.0, 0.0 },
		{ position.x, position.y, position.z, 1.0 }
	};

	return MMatrix(matrix);

};


MStatus PathConstraint::createRotationMatrix(int forwardAxis, MAngle angle, MMatrix &matrix)
/**
Creates a rotation matrix from the given forward axis and angle.

@param forwardAxis: The forward axis to rotate from.
@param angle: The angle of rotation.
@param matrix: The passed matrix to populate.
@return: Return status.
*/
{

	double radian = angle.asRadians();

	switch (forwardAxis)
	{

		case 0: case 1:

			matrix = MEulerRotation(radian, 0.0, 0.0).asMatrix();
			break;

		case 2: case 3:

			matrix = MEulerRotation(0.0, radian, 0.0).asMatrix();
			break;

		case 4: case 5:

			matrix = MEulerRotation(0.0, 0.0, radian).asMatrix();
			break;

		default:

			return MS::kFailure;

	}

	return MS::kSuccess;

};


MStatus PathConstraint::getUpVector(MDataBlock& data, double parameter, MObject& curve, MVector& upVector)
/**
Returns the up vector based on the selected world up type.

@param data: Data block containing storage for the node's attributes.
@param parameter: The curve parameter to sample at.
@param curve: The curve data object to sample from.
@param upVector: The passed vector to populate.
@return: Return status.
*/
{

	MStatus status;

	// Get up vector related data handles
	//
	MDataHandle worldUpTypeHandle = data.inputValue(PathConstraint::worldUpType, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MDataHandle worldUpVectorHandle = data.inputValue(PathConstraint::worldUpVector, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MDataHandle worldUpMatrixHandle = data.inputValue(PathConstraint::worldUpMatrix, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Get values from handles
	//
	int worldUpType = worldUpTypeHandle.asShort();
	MVector worldUpVector = worldUpVectorHandle.asVector();
	MMatrix worldUpMatrix = worldUpMatrixHandle.asMatrix();

	// Check world up type
	//
	switch (worldUpType)
	{

		case 0: // Scene Up
		{

			upVector = PathConstraint::getSceneUpVector();

		}
		break;

		case 1: // Object Up
		{

			upVector = this->getObjectUpVector(worldUpMatrix);

		}
		break;

		case 2: // Object Rotation Up
		{

			upVector = PathConstraint::getObjectRotationUpVector(worldUpMatrix, worldUpVector);

		}
		break;

		case 3: // Vector
		{

			upVector = MVector(worldUpVector).normal();

		}
		break;

		case 4: // Curve normal
		{

			status = PathConstraint::getCurveNormal(curve, parameter, upVector);
			CHECK_MSTATUS_AND_RETURN_IT(status);

		}
		break;

		default:
		{

			return MS::kFailure;

		}
		break;

	}

	return MS::kSuccess;

};


MStatus PathConstraint::getForwardVector(MObject& curve, double parameter, MVector& forwardVector)
/**
Returns the forward vector at the given percentile.

@param curve: The curve data object to sample from.
@param parameter: The curve parameter to sample at.
@param forwardVector: The passed vector to populate.
@return: Return status.
*/
{

	MStatus status;

	// Initialize function set
	//
	MFnNurbsCurve fnCurve(curve, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Return tagent vector at parameter
	//
	forwardVector = fnCurve.tangent(parameter, MSpace::kWorld, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = forwardVector.normalize();
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MS::kSuccess;

};


MVector PathConstraint::getSceneUpVector()
/**
Returns the scene up vector.

@return: MVector
*/
{

	return MGlobal::upAxis();

};


MVector PathConstraint::getObjectUpVector(MMatrix worldUpMatrix)
/**
Returns the vector between this object and the supplied matrix.

@param worldUpMatrix: The world matrix of the up object.
@return: MVector
*/
{

	// Get dag path to this constraint
	//
	MDagPath dagPath;
	MDagPath::getAPathTo(this->thisMObject(), dagPath);

	// Return point in constraint space
	//
	MMatrix inclusiveMatrixInverse = dagPath.inclusiveMatrixInverse();
	MMatrix matrix = worldUpMatrix * inclusiveMatrixInverse;

	return MVector(matrix[3]).normal();

};


MVector PathConstraint::getObjectRotationUpVector(MMatrix worldUpMatrix, MVector worldUpVector)
/**
Returns the weighted average for the up vector derived from the supplied world matrix.

@param worldUpMatrix: The world matrix of the up object.
@param worldUpVector: The weighted values to average from.
@return: MVector
*/
{

	// Extract axis vectors
	//
	MVector xAxis = MVector(worldUpMatrix[0]);
	MVector yAxis = MVector(worldUpMatrix[1]);
	MVector zAxis = MVector(worldUpMatrix[2]);

	// Calculate weighted vector average
	//
	return MVector((xAxis * worldUpVector.x) + (yAxis * worldUpVector.y) + (zAxis * worldUpVector.z)).normal();

};


MStatus	PathConstraint::getCurvePoint(MObject& curve, double parameter, MPoint& position)
/**
Returns the point on a curve from the given percentile.

@param curve: The curve data object to sample from.
@param parameter: The curve parameter to sample at.
@param position: The passed point to populate.
@return: Return status.
*/
{

	MStatus status;

	// Initialize function set
	//
	MFnNurbsCurve fnCurve(curve, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Return normal at parameter
	//
	status = fnCurve.getPointAtParam(parameter, position, MSpace::kWorld);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return status;

};


MStatus	PathConstraint::getCurveNormal(MObject& curve, double parameter, MVector& upVector)
/**
Returns the tangent normal at the given percentile.

@param curve: The curve data object to sample from.
@param parameter: The curve parameter to sample at.
@param upVector: The passed vector to populate.
@return: Return status.
*/
{
	
	MStatus status;

	// Initialize function set
	//
	MFnNurbsCurve fnCurve(curve, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Return normal at parameter
	//
	upVector = fnCurve.normal(parameter, MSpace::kWorld, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return status;

};


MStatus PathConstraint::getParamFromFraction(MObject& curve, double fraction, double& parameter)
/**
Returns the curve parameter based on the fractional distance across the curve.

@param curve: The curve data object to sample from.
@param fraction: The fractional distance, between 0-1, to sample from.
@param parameter: The passed parameter to populate.
@return: Return status.
*/
{

	MStatus status;

	// Initialize function set
	//
	MFnNurbsCurve fnCurve(targetCurve, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Calculate the fractional distance
	// Also don't forget to clamp the value!!!
	//
	fraction = PathConstraint::clamp(fraction, 0.0, 1.0);

	double curveLength = fnCurve.length();
	double paramLength = fraction * curveLength;

	if (paramLength == 0.0)
	{

		paramLength += 0.001;

	}
	else if (paramLength == curveLength)
	{

		paramLength -=  0.001;

	}
	else;

	// Get param from length
	//
	parameter = fnCurve.findParamFromLength(paramLength);

	return MS::kSuccess;

};


MStatus PathConstraint::composeMatrix(int forwardAxis, MVector forwardVector, int upAxis, MVector upVector, MPoint position, MMatrix &matrix)
/**
Composes a matrix the given forward/up vector and position.
Axis enumerator values can be supplied to designate the vectors.

@param forwardAxis: The forward axis to assign the forward vector to.
@param forwardVector: The forward vector of the matrix.
@param upAxis: The up axis to assign the up vector to.
@param upVector: Tangent vector to resolve the last remaining axis.
@param matrix: The passed matrix to populate.
@return: Return status.
*/
{

	MStatus status;

	// Declare axis vectors
	//
	MVector x, y, z;

	switch (forwardAxis)
	{

		case 0: // +x
		{

			// Assign forward vector
			//
			x = forwardVector;

			// Calculate remaining axises
			//
			if (upAxis == 0 || upAxis == 1)
			{

				return MS::kFailure;

			}
			else if (upAxis == 2 || upAxis == 3)
			{

				z = (x ^ upVector).normal();
				y = (z ^ x).normal();

			}
			else if (upAxis == 4 || upAxis == 5)
			{

				y = (upVector ^ x).normal();
				z = (x ^ y).normal();

			}

		}
		break;

		case 1: // -x
		{

			// Assign forward vector
			//
			x = forwardVector;

			// Calculate remaining axises
			//
			if (upAxis == 0 || upAxis == 1)
			{

				return MS::kFailure;

			}
			else if (upAxis == 2 || upAxis == 3)
			{

				z = (x ^ upVector).normal();
				y = (z ^ x).normal();

			}
			else if (upAxis == 4 || upAxis == 5)
			{

				y = (upVector ^ x).normal();
				z = (x ^ y).normal();

			}

		}
		break;

		case 2: // +y
		{

			// Assign forward vector
			//
			y = forwardVector;

			// Calculate remaining axises
			//
			if (upAxis == 0 || upAxis == 1)
			{

				z = (upVector ^ y).normal();
				x = (y ^ z).normal();

			}
			else if (upAxis == 2 || upAxis == 3)
			{

				return MS::kFailure;

			}
			else if (upAxis == 4 || upAxis == 5)
			{

				x = (y ^ upVector).normal();
				z = (x ^ y).normal();

			}

		}
		break;

		case 3: // -y
		{

			// Assign forward vector
			//
			y = forwardVector;

			// Calculate remaining axises
			//
			if (upAxis == 0 || upAxis == 1)
			{

				z = (upVector ^ y).normal();
				x = (y ^ z).normal();

			}
			else if (upAxis == 2 || upAxis == 3)
			{

				return MS::kFailure;

			}
			else if (upAxis == 4 || upAxis == 5)
			{

				x = (y ^ upVector).normal();
				z = (x ^ y).normal();

			}

		}
		break;

		case 4: // +z
		{

			// Assign forward vector
			//
			z = forwardVector;

			// Calculate remaining axises
			//
			if (upAxis == 0 || upAxis == 1)
			{

				y = (z ^ upVector).normal();
				x = (y ^ z).normal();

			}
			else if (upAxis == 2 || upAxis == 3)
			{

				x = (upVector ^ z).normal();
				y = (z ^ x).normal();

			}
			else if (upAxis == 4 || upAxis == 5)
			{

				return MS::kFailure;

			}

		}
		break;

		case 5: // -z
		{

			// Assign forward vector
			//
			z = forwardVector;

			// Calculate remaining axises
			//
			if (upAxis == 0 || upAxis == 1)
			{

				y = (z ^ upVector).normal();
				x = (y ^ z).normal();

			}
			else if (upAxis == 2 || upAxis == 3)
			{

				x = (upVector ^ z).normal();
				y = (z ^ x).normal();

			}
			else if (upAxis == 4 || upAxis == 5)
			{

				return MS::kFailure;

			}

		}
		break;

		default:
		{

			return MS::kFailure;

		}
		break;

	}

	// Return matrix from axis vectors
	//
	matrix = PathConstraint::composeMatrix(x, y, z, position);

	return MS::kSuccess;

};


MMatrix PathConstraint::composeMatrix(MVector x, MVector y, MVector z, MPoint p)
/**
Composes a matrix from the axis vectors and position.

@param x: The X axis vector.
@param y: The Y axis vector.
@param z: The Z axis vector.
@param p: The position.
@return: MMatrix
*/
{

	const double matrixList[4][4] = {
		{x.x, x.y, x.z, 0.0},
		{y.x, y.y, y.z, 0.0},
		{z.x, z.y, z.z, 0.0},
		{p.x, p.y, p.z, 1.0}
	};

	return MMatrix(matrixList);

};


template<class N> N lerp(N start, N end, double weight)
/**
Linearly interpolates the two given numbers using the supplied weight.

@param start: The start number.
@param end: The end number.
@param weight: The amount to blend.
@return: The interpolated value.
*/
{

	return (start * (1.0 - weight)) + (end * weight);

};


MMatrix PathConstraint::blendMatrices(MMatrix startMatrix, MMatrix endMatrix, float weight)
/**
Interpolates the two given matrices using the supplied weight.
Both translate and scale will be lerp'd while rotation will be slerp'd.

@param startMatrix: The start matrix.
@param endMatrix: The end matrix.
@param weight: The amount to blend.
@return: The interpolated matrix.
*/
{

	// Initialize transformation matrices
	//
	MTransformationMatrix startTransformationMatrix = MTransformationMatrix(startMatrix);
	MTransformationMatrix endTransformationMatrix = MTransformationMatrix(endMatrix);

	// Interpolate translation
	//
	MVector startTranslation = startTransformationMatrix.getTranslation(MSpace::kTransform);
	MVector endTranslation = endTransformationMatrix.getTranslation(MSpace::kTransform);

	MVector translation = lerp(startTranslation, endTranslation, weight);

	// Interpolate rotation
	//
	MQuaternion startQuat = startTransformationMatrix.rotation();
	MQuaternion endQuat = endTransformationMatrix.rotation();

	MQuaternion quat = PathConstraint::slerp(startQuat, endQuat, weight);

	// Compose interpolated matrix
	//
	MMatrix translateMatrix = PathConstraint::createPositionMatrix(translation);
	MMatrix rotateMatrix = quat.asMatrix();

	return rotateMatrix * translateMatrix;

};


MMatrix PathConstraint::blendMatrices(MMatrix restMatrix, MMatrixArray matrices, MFloatArray weights)
/**
Interpolates the supplied matrices using the weight array as a blend aplha.
The rest matrix is used just in case the weights don't equal 1.

@param restMatrix: The default matrix to blend from in case the weights don't equal 1.
@param matrices: The matrix array to blend.
@param weights: The float array containing the weighted averages.
@return: The interpolated matrix.
*/
{

	// Check the number of matrices
	//
	unsigned int numMatrices = matrices.length();

	switch (numMatrices)
	{

		case 0:
		{

			// Reuse rest matrix
			//
			return MMatrix(restMatrix);

		}
		break;

		case 1:
		{

			// Check weight sum before committing to either matrix
			//
			float weightSum = PathConstraint::sum(weights);

			if (weightSum == 1.0f)
			{

				return MMatrix(matrices[0]);

			}
			else if (weightSum == 0.f)
			{

				return MMatrix(restMatrix);

			}
			else
			{

				return PathConstraint::blendMatrices(restMatrix, matrices[0], weights[0]);

			}

		}
		break;

		default:
		{

			// Get start matrix
			//
			MFloatArray clampedWeights = PathConstraint::clamp(weights);
			float weightSum = PathConstraint::sum(clampedWeights);

			MMatrix matrix = MMatrix(matrices[0]);

			if (weightSum < 1.0f)
			{

				matrix = MMatrix(restMatrix);

			}

			// Get start transform components
			//
			unsigned int numMatrices = matrices.length();

			for (unsigned int i = 0; i < numMatrices; i++)
			{

				matrix = PathConstraint::blendMatrices(matrix, matrices[i], weights[i]);

			}

			return matrix;

		}
		break;

	}

	return MMatrix::identity;

};


MQuaternion PathConstraint::slerp(MQuaternion startQuat, MQuaternion endQuat, float weight)
/**
Spherical interpolates two quaternions.

@param startQuat: Start Quaternion.
@param endQuat: End Quaternion.
@param weight: The amount to interpolate.
@return: The interpolated quaternion.
*/
{

	// Calculate angle between quats
	// If startQuat == endQuat or startQuat == -endQuat then theta = 0 and we can return startQuat
	//
	double cos_half_theta = startQuat.w * endQuat.w + startQuat.x * endQuat.x + startQuat.y * endQuat.y + startQuat.z * endQuat.z;

	if (abs(cos_half_theta) >= 1.0)
	{

		return MQuaternion(startQuat);

	}

	// Calculate temporary values
	// If theta = 180 degrees then result is not fully defined
	// We could rotate around any axis normal to startQuat or endQuat
	//
	MQuaternion quat = MQuaternion();

	double half_theta = acos(cos_half_theta);
	double sin_half_theta = sqrt(1.0 - cos_half_theta * cos_half_theta);

	if (fabs(sin_half_theta) < 0.001)
	{

		quat.x = startQuat.x * 0.5 + endQuat.x * 0.5;
		quat.y = startQuat.y * 0.5 + endQuat.y * 0.5;
		quat.z = startQuat.z * 0.5 + endQuat.z * 0.5;
		quat.w = startQuat.w * 0.5 + endQuat.w * 0.5;

		return quat;

	}

	// Calculate quaternion
	//
	double ratio_a = sin((1.0 - weight) * half_theta) / sin_half_theta;
	double ratio_b = sin(weight * half_theta) / sin_half_theta;

	quat.w = startQuat.w * ratio_a + endQuat.w * ratio_b;
	quat.x = startQuat.x * ratio_a + endQuat.x * ratio_b;
	quat.y = startQuat.y * ratio_a + endQuat.y * ratio_b;
	quat.z = startQuat.z * ratio_a + endQuat.z * ratio_b;

	return quat;

}


float PathConstraint::sum(MFloatArray items)
/**
Calculates the sum of all the supplied items.

@param items: The items to add up.
@return: The total sum.
*/
{

	// Iterate through numbers
	//
	unsigned int numItems = items.length();
	float sum = 0.0;

	for (unsigned int i = 0; i < numItems; i++)
	{

		sum += items[i];

	}

	return sum;

};


MFloatArray PathConstraint::clamp(MFloatArray items)
/**
Clamps the supplied items so they don't exceed 1.
Anything below that is left alone and compensated for using the rest matrix.

@param items: The float array containing the weighted averages.
@return: The newly clamped array of weights.
*/
{

	// Check if sum is greater than one
	//
	float sum = PathConstraint::sum(items);

	if (sum < 1.0)
	{

		return MFloatArray(items);

	}

	// Iterate through numbers
	//
	float fraction = 1.0f / sum;

	unsigned int numItems = items.length();
	MFloatArray normalizedItems = MFloatArray(numItems);

	for (unsigned int i = 0; i < numItems; i++)
	{

		normalizedItems[i] = items[i] * fraction;

	}

	return normalizedItems;


};


double PathConstraint::clamp(double value, double min, double max)
/**
Clamps the supplied value based on the specified range.

@param value: The value to clamp.
@param min: The minimum value.
@param max: The maximum value.
@return: The clamped value.
*/
{

	if (value < min)
	{

		return min;

	}
	else if (value > min)
	{

		return max;

	}
	else;

	return value;

};


const MObject PathConstraint::targetAttribute() const
/**
Returns the target attribute for the constraint.
Default implementation returns MObject::kNullObj.

@return: MObject
*/
{


	return PathConstraint::target;

};


const MObject PathConstraint::weightAttribute() const
/**
Returns the weight attribute for the constraint.
Default implementation returns MObject::kNullObj.

@return: MObject
*/
{


	return PathConstraint::targetWeight;

};


const MObject PathConstraint::constraintRotateOrderAttribute() const
/**
Returns the rotate order attribute for the constraint.
Default implementation returns MObject::kNullObj.

@return: MObject
*/
{


	return PathConstraint::constraintRotateOrder;

};


void* PathConstraint::creator()
/**
This function is called by Maya when a new instance is requested.
See pluginMain.cpp for details.

@return: PathConstraint
*/
{

	return new PathConstraint();

}


void PathConstraint::postConstructor()
/**
Internally maya creates two objects when a user defined node is created, the internal MObject and the user derived object.
The association between the these two objects is not made until after the MPxNode constructor is called.
This implies that no MPxNode member function can be called from the MPxNode constructor.
The postConstructor will get called immediately after the constructor when it is safe to call any MPxNode member function.

@return: void
*/
{

	// Store reference to this instance
	//
	this->instances.insert(std::make_pair(this->hashCode(), this));

};


long PathConstraint::hashCode()
/**
Returns the hash code for this instance.

@return: Hash code.
*/
{

	return MObjectHandle(this->thisMObject()).hashCode();

};


MStatus PathConstraint::initialize()
/**
This function is called by Maya after a plugin has been loaded.
Use this function to define any static attributes.

@return: MStatus
*/
{

	MStatus	status;

	// Declare attribute function sets
	//
	MFnCompoundAttribute fnCompoundAttr;
	MFnNumericAttribute fnNumericAttr;
	MFnUnitAttribute fnUnitAttr;
	MFnTypedAttribute fnTypedAttr;
	MFnEnumAttribute fnEnumAttr;
	MFnMatrixAttribute fnMatrixAttr;
	MFnMessageAttribute fnMessageAttr;

	// Input attributes:
	// ".restTranslateX" attribute
	//
	PathConstraint::restTranslateX = fnUnitAttr.create("restTranslateX", "rtx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".restTranslateY" attribute
	//
	PathConstraint::restTranslateY = fnUnitAttr.create("restTranslateY", "rty", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".restTranslateZ" attribute
	//
	PathConstraint::restTranslateZ = fnUnitAttr.create("restTranslateZ", "rtz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".restTranslate" attribute
	//
	PathConstraint::restTranslate = fnNumericAttr.create("restTranslate", "rt", PathConstraint::restTranslateX, PathConstraint::restTranslateY, PathConstraint::restTranslateZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".restRotateX" attribute
	//
	PathConstraint::restRotateX = fnUnitAttr.create("restRotateX", "rrx", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".restRotateY" attribute
	//
	PathConstraint::restRotateY = fnUnitAttr.create("restRotateY", "rry", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".restRotateZ" attribute
	//
	PathConstraint::restRotateZ = fnUnitAttr.create("restRotateZ", "rrz", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".restRotate" attribute
	//
	PathConstraint::restRotate = fnNumericAttr.create("restRotate", "rr", PathConstraint::restRotateX, PathConstraint::restRotateY, PathConstraint::restRotateZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".uValue" attribute
	//
	PathConstraint::uValue = fnNumericAttr.create("uValue", "u", MFnNumericData::kDouble, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".fractionMode" attribute
	//
	PathConstraint::fractionMode = fnNumericAttr.create("fractionMode", "fm", MFnNumericData::kBoolean, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".forwardAxis" attribute
	//
	PathConstraint::forwardAxis = fnEnumAttr.create("forwardAxis", "fa", short(0), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("PosX", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("NegX", 1));
	CHECK_MSTATUS(fnEnumAttr.addField("PosY", 2));
	CHECK_MSTATUS(fnEnumAttr.addField("NegY", 3));
	CHECK_MSTATUS(fnEnumAttr.addField("PosZ", 4));
	CHECK_MSTATUS(fnEnumAttr.addField("NegZ", 5));
	
	// ".forwardTwist" attribute
	//
	PathConstraint::forwardTwist = fnUnitAttr.create("forwardTwist", "ft", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".upAxis" attribute
	//
	PathConstraint::upAxis = fnEnumAttr.create("upAxis", "ua", short(2), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("PosX", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("NegX", 1));
	CHECK_MSTATUS(fnEnumAttr.addField("PosY", 2));
	CHECK_MSTATUS(fnEnumAttr.addField("NegY", 3));
	CHECK_MSTATUS(fnEnumAttr.addField("PosZ", 4));
	CHECK_MSTATUS(fnEnumAttr.addField("NegZ", 5));

	// ".worldUpType" attribute
	//
	PathConstraint::worldUpType = fnEnumAttr.create("worldUpType", "wut", short(0), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("Scene Up", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("Object Up", 1));
	CHECK_MSTATUS(fnEnumAttr.addField("Object Rotation Up", 2));
	CHECK_MSTATUS(fnEnumAttr.addField("Vector", 3));
	CHECK_MSTATUS(fnEnumAttr.addField("Normal", 4));

	// ".worldUpVectorX" attribute
	//
	PathConstraint::worldUpVectorX = fnUnitAttr.create("worldUpVectorX", "wuvx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".worldUpVectorY" attribute
	//
	PathConstraint::worldUpVectorY = fnUnitAttr.create("worldUpVectorY", "wuvy", MFnUnitAttribute::kDistance, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".worldUpVectorZ" attribute
	//
	PathConstraint::worldUpVectorZ = fnUnitAttr.create("worldUpVectorZ", "wuvz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".worldUpVector" attribute
	//
	PathConstraint::worldUpVector = fnNumericAttr.create("worldUpVector", "wuv", PathConstraint::worldUpVectorX, PathConstraint::worldUpVectorY, PathConstraint::worldUpVectorZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".worldUpMatrix" attribute
	//
	PathConstraint::worldUpMatrix = fnMatrixAttr.create("worldUpMatrix", "wum", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".constraintRotateOrder" attribute
	//
	PathConstraint::constraintRotateOrder = fnEnumAttr.create("constraintRotateOrder", "cro", short(0), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("xyz", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("yzx", 1));
	CHECK_MSTATUS(fnEnumAttr.addField("zxy", 2));
	CHECK_MSTATUS(fnEnumAttr.addField("xzy", 3));
	CHECK_MSTATUS(fnEnumAttr.addField("yxz", 4));
	CHECK_MSTATUS(fnEnumAttr.addField("zyx", 5));

	// ".constraintParentInverseMatrix" attribute
	//
	PathConstraint::constraintParentInverseMatrix = fnMatrixAttr.create("constraintParentInverseMatrix", "cpim", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Target Attributes:
	// Define ".targetWeight" attribute
	//
	PathConstraint::targetWeight = fnNumericAttr.create("targetWeight", "tw", MFnNumericData::kFloat, 0.0f, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setMin(0.0));
	CHECK_MSTATUS(fnNumericAttr.setMax(1.0));

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PathConstraint::targetCategory));

	// ".targetCurve" attribute
	//
	PathConstraint::targetCurve = fnTypedAttr.create("targetCurve", "tc", MFnData::kNurbsCurve, MObject::kNullObj, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnTypedAttr.addToCategory(PathConstraint::targetCategory));

	// Define ".target" attribute
	//
	PathConstraint::target = fnCompoundAttr.create("target", "tg", &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnCompoundAttr.addChild(PathConstraint::targetWeight));
	CHECK_MSTATUS(fnCompoundAttr.addChild(PathConstraint::targetCurve));

	CHECK_MSTATUS(fnCompoundAttr.setArray(true));

	// Output attributes:
	// Define ".constraintTranslateX" attribute
	//
	PathConstraint::constraintTranslateX = fnUnitAttr.create("constraintTranslateX", "ctx", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PathConstraint::outputCategory));

	// ".constraintTranslateY" attribute
	//
	PathConstraint::constraintTranslateY = fnUnitAttr.create("constraintTranslateY", "cty", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PathConstraint::outputCategory));

	// ".constraintTranslateZ" attribute
	//
	PathConstraint::constraintTranslateZ = fnUnitAttr.create("constraintTranslateZ", "ctz", MFnUnitAttribute::kDistance, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PathConstraint::outputCategory));

	// ".constraintTranslate" attribute
	//
	PathConstraint::constraintTranslate = fnNumericAttr.create("constraintTranslate", "ct", PathConstraint::constraintTranslateX, PathConstraint::constraintTranslateY, PathConstraint::constraintTranslateZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PathConstraint::outputCategory));

	// ".constraintRotateX" attribute
	//
	PathConstraint::constraintRotateX = fnUnitAttr.create("constraintRotateX", "crx", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PathConstraint::outputCategory));

	// ".constraintRotateY" attribute
	//
	PathConstraint::constraintRotateY = fnUnitAttr.create("constraintRotateY", "cry", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PathConstraint::outputCategory));

	// ".constraintRotateZ" attribute
	//
	PathConstraint::constraintRotateZ = fnUnitAttr.create("constraintRotateZ", "crz", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));

	CHECK_MSTATUS(fnUnitAttr.addToCategory(PathConstraint::outputCategory));

	// ".constraintRotate" attribute
	//
	PathConstraint::constraintRotate = fnNumericAttr.create("constraintRotate", "cr", PathConstraint::constraintRotateX, PathConstraint::constraintRotateY, PathConstraint::constraintRotateZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));

	CHECK_MSTATUS(fnNumericAttr.addToCategory(PathConstraint::outputCategory));

	// ".constraintMatrix" attribute
	//
	PathConstraint::constraintMatrix = fnMatrixAttr.create("constraintMatrix", "cm", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.setWritable(false));
	CHECK_MSTATUS(fnMatrixAttr.setStorable(false));

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(PathConstraint::outputCategory));

	// ".constraintInverseMatrix" attribute
	//
	PathConstraint::constraintInverseMatrix = fnMatrixAttr.create("constraintInverseMatrix", "cim", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.setWritable(false));
	CHECK_MSTATUS(fnMatrixAttr.setStorable(false));

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(PathConstraint::outputCategory));

	// ".constraintWorldMatrix" attribute
	//
	PathConstraint::constraintWorldMatrix = fnMatrixAttr.create("constraintWorldMatrix", "cwm", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.setWritable(false));
	CHECK_MSTATUS(fnMatrixAttr.setStorable(false));

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(PathConstraint::outputCategory));

	// ".constraintWorldInverseMatrix" attribute
	//
	PathConstraint::constraintWorldInverseMatrix = fnMatrixAttr.create("constraintWorldInverseMatrix", "cwim", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.setWritable(false));
	CHECK_MSTATUS(fnMatrixAttr.setStorable(false));

	CHECK_MSTATUS(fnMatrixAttr.addToCategory(PathConstraint::outputCategory));

	// ".constraintObject" attribute
	//
	PathConstraint::constraintObject = fnMessageAttr.create("constraintObject", "co", &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMessageAttr.setWritable(true));
	CHECK_MSTATUS(fnMessageAttr.setStorable(true));

	// Add attributes
	//
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::restTranslate));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::restRotate));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::uValue));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::fractionMode));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::forwardAxis));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::forwardTwist));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::upAxis));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::worldUpType));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::worldUpVector));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::worldUpMatrix));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::target));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::constraintTranslate));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::constraintRotate));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::constraintRotateOrder));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::constraintMatrix));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::constraintInverseMatrix));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::constraintWorldMatrix));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::constraintWorldInverseMatrix));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::constraintParentInverseMatrix));
	CHECK_MSTATUS(PathConstraint::addAttribute(PathConstraint::constraintObject));

	// Define target attribute relationships
	//
	status = fnCompoundAttr.setObject(PathConstraint::target);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	unsigned int numChildren = fnCompoundAttr.numChildren();

	for (unsigned int i = 0; i < numChildren; i++) 
	{

		MObject child = fnCompoundAttr.child(i);

		CHECK_MSTATUS(PathConstraint::attributeAffects(child, PathConstraint::constraintTranslate));
		CHECK_MSTATUS(PathConstraint::attributeAffects(child, PathConstraint::constraintTranslateX));
		CHECK_MSTATUS(PathConstraint::attributeAffects(child, PathConstraint::constraintTranslateY));
		CHECK_MSTATUS(PathConstraint::attributeAffects(child, PathConstraint::constraintTranslateZ));

		CHECK_MSTATUS(PathConstraint::attributeAffects(child, PathConstraint::constraintRotate));
		CHECK_MSTATUS(PathConstraint::attributeAffects(child, PathConstraint::constraintRotateX));
		CHECK_MSTATUS(PathConstraint::attributeAffects(child, PathConstraint::constraintRotateY));
		CHECK_MSTATUS(PathConstraint::attributeAffects(child, PathConstraint::constraintRotateZ));

		CHECK_MSTATUS(PathConstraint::attributeAffects(child, PathConstraint::constraintMatrix));
		CHECK_MSTATUS(PathConstraint::attributeAffects(child, PathConstraint::constraintInverseMatrix));
		CHECK_MSTATUS(PathConstraint::attributeAffects(child, PathConstraint::constraintWorldMatrix));
		CHECK_MSTATUS(PathConstraint::attributeAffects(child, PathConstraint::constraintWorldInverseMatrix));

	}

	// Define rest attribute relationships
	//
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::restTranslate, PathConstraint::constraintTranslate));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::restTranslateX, PathConstraint::constraintTranslateX));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::restTranslateY, PathConstraint::constraintTranslateY));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::restTranslateZ, PathConstraint::constraintTranslateZ));

	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::restRotate, PathConstraint::constraintRotate));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::restRotateX, PathConstraint::constraintRotateX));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::restRotateY, PathConstraint::constraintRotateY));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::restRotateZ, PathConstraint::constraintRotateZ));

	// Define constraint attribute relationships
	//
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::uValue, PathConstraint::constraintTranslate));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::uValue, PathConstraint::constraintTranslateX));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::uValue, PathConstraint::constraintTranslateY));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::uValue, PathConstraint::constraintTranslateZ));
	
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::fractionMode, PathConstraint::constraintTranslate));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::fractionMode, PathConstraint::constraintTranslateX));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::fractionMode, PathConstraint::constraintTranslateY));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::fractionMode, PathConstraint::constraintTranslateZ));

	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::forwardTwist, PathConstraint::constraintRotate));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::forwardTwist, PathConstraint::constraintRotateX));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::forwardTwist, PathConstraint::constraintRotateY));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::forwardTwist, PathConstraint::constraintRotateZ));

	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintRotateOrder, PathConstraint::constraintRotate));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintRotateOrder, PathConstraint::constraintRotateX));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintRotateOrder, PathConstraint::constraintRotateY));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintRotateOrder, PathConstraint::constraintRotateZ));

	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintParentInverseMatrix, PathConstraint::constraintTranslate));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintParentInverseMatrix, PathConstraint::constraintTranslateX));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintParentInverseMatrix, PathConstraint::constraintTranslateY));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintParentInverseMatrix, PathConstraint::constraintTranslateZ));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintParentInverseMatrix, PathConstraint::constraintRotate));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintParentInverseMatrix, PathConstraint::constraintRotateX));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintParentInverseMatrix, PathConstraint::constraintRotateY));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintParentInverseMatrix, PathConstraint::constraintRotateZ));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintParentInverseMatrix, PathConstraint::constraintMatrix));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintParentInverseMatrix, PathConstraint::constraintInverseMatrix));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintParentInverseMatrix, PathConstraint::constraintWorldMatrix));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::constraintParentInverseMatrix, PathConstraint::constraintWorldInverseMatrix));

	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::worldUpType, PathConstraint::constraintRotate));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::worldUpType, PathConstraint::constraintRotateX));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::worldUpType, PathConstraint::constraintRotateY));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::worldUpType, PathConstraint::constraintRotateZ));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::worldUpVector, PathConstraint::constraintRotate));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::worldUpVector, PathConstraint::constraintRotateX));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::worldUpVector, PathConstraint::constraintRotateY));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::worldUpVector, PathConstraint::constraintRotateZ));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::worldUpMatrix, PathConstraint::constraintRotate));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::worldUpMatrix, PathConstraint::constraintRotateX));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::worldUpMatrix, PathConstraint::constraintRotateY));
	CHECK_MSTATUS(PathConstraint::attributeAffects(PathConstraint::worldUpMatrix, PathConstraint::constraintRotateZ));

	// Create child added callback
	//
	PathConstraint::childAddedCallbackId = MDagMessage::addChildAddedCallback(onChildAdded);

	return status;

};