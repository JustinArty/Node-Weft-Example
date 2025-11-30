#include "OpticsNode.h"

OpticsData& OpticsData::operator+=(const OpticsData& b)
{
	rays.insert(rays.end(), b.rays.begin(), b.rays.end());
	lenses.insert(lenses.end(), b.lenses.begin(), b.lenses.end());
	return *this;
}

void OpticsData::displayOnViewport(NodeAssistUI& ui)
{
	for (auto& r : rays) {
		AssistPlot2D::Line line;
		line.color = r.getWavelengthColor();
		line.points = r.path;
		line.extendDirection = r.direction;
		ui.assistPlot2D.lines.push_back(line);
	}
	for (auto& l : lenses) {
		AssistPlot2D::Line line;
		const int segments = 36;
		Vec2 center = l.center + Vec2{ l.radius,0 };
		for (int i = 0; i <= segments; i++) {
			double angle = (double)i / segments * 3.14159;
			Vec2 pointOnLens = center + Vec2{ -sin(angle),cos(angle) } * l.radius;
			line.points.push_back(pointOnLens);
		}
		line.color = tRGB{ 150,150,250 };
		ui.assistPlot2D.lines.push_back(line);
	}

}

bool OpticsSourceNode::bake()
{
	output.clear();
	if (inputNode[0] != nullptr) {
		if (dynamic_pointer_cast<OpticsSourceNode>(inputNode[0].lock()) == nullptr)
			return false; // only accept source node as input, otherwise show error
		else
			output += inputNode[0]->getOutput();
	}

	// generate rays
	for (int i = 0; i < rayCount; i++) {
		Vec2 aperturePoint = { apertureX, rayCount != 1 ? ((double)i / (rayCount - 1) - 0.5) * apertureSize : 0 };
		Vec2 startPoint;
		if (sourceType == PointSource)
			startPoint = center;
		else {
			double angleRad = parallelAngle * 3.14159 / 180.0;
			startPoint = Vec2{ -parallelStartOffset * cos(angleRad), -parallelStartOffset * sin(angleRad) } + aperturePoint;
		}
		Ray r(startPoint, aperturePoint - startPoint, wavelength);
		oOutput->data.rays.push_back(r);	
	}
    return true;
}

bool OpticsLensNode::bake()
{
	output.clear();
	if (inputNode[0] != nullptr) {
		if(dynamic_cast<OpticsLensNode*>(inputNode[0].get()) == nullptr)
			return false; // only accept source/lens node as input, otherwise show error
		else
			output += inputNode[0]->getOutput();
	}

	// add lens
	SphereLens lens{ Vec2{ positionX, 0 }, curvatureRadius, refractiveIndex, isEntrance, dispersiveCoefficient };
	oOutput->data.lenses.push_back(lens);
	return true;
}

bool RefractNode::bake()
{
	output.clear();
	setPinInfo(0, L"light source");
	setPinInfo(1, L"Lenses");

	// check inputs
	if (inputNode[0] == nullptr || inputNode[0]->getOutput<OpticsNodeOutputData>()->data.rays.empty()) {
		setWarningFlag(true);
		setUIInfo(L"No input light data");
		return true;
	}
	if (inputNode[1] == nullptr || inputNode[1]->getOutput<OpticsNodeOutputData>()->data.lenses.empty()) {
		setWarningFlag(true);
		setUIInfo(L"No input lens data");
		return true;
	}
	
	// get input data
	vector<Ray> rays = inputNode[0]->getOutput<OpticsNodeOutputData>()->data.rays;
	vector<SphereLens> lenses = inputNode[1]->getOutput<OpticsNodeOutputData>()->data.lenses;

	// process rays through lenses
	for (auto& r : rays) {
		double refractiveIndexBefore = 1.0; // assume air
		for (auto& l : lenses) {
			if (intersectAndUpdateRay(r, l, refractiveIndexBefore)) {
				refractRay(r, l);
				if (l.isEntrance)
					refractiveIndexBefore = l.getRefractiveIndexAtWavelength(r.wavelength);
				else
					refractiveIndexBefore = 1.0;
			}
		}
	}
	oOutput->data.rays = rays;
	oOutput->data.lenses = lenses;

	return true;
}
