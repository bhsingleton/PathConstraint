//
// File: pluginMain.cpp
//
// Author: Benjamin H. Singleton
//

#include "PathConstraint.h"
#include <maya/MFnPlugin.h>


MStatus initializePlugin(MObject obj)
{

	MStatus status;

	MFnPlugin plugin(obj, "Ben Singleton", "2020", "Any");
	status = plugin.registerNode("pathConstraint", PathConstraint::id, PathConstraint::creator, PathConstraint::initialize, MPxNode::kConstraintNode);

	if (!status)
	{

		status.perror("registerNode");
		return status;

	}

	return status;

}


MStatus uninitializePlugin(MObject obj)
{

	MStatus status;

	MFnPlugin plugin(obj);
	status = plugin.deregisterNode(PathConstraint::id);

	if (!status)
	{

		status.perror("deregisterNode");
		return status;

	}

	return status;

}
