#pragma once
#include "BaseNodeType.h"
#include "RayOptics.h"

using namespace NodeWeft;
using namespace Optics;
using namespace std;

struct OpticsData {
	vector<Ray> rays;
	vector<SphereLens> lenses;

	OpticsData& operator+=(const OpticsData& b);
	void displayOnViewport(NodeAssistUI& ui);
};

using OpticsNodeOutputData = SimpleNodeOutputData<OpticsData>;

template<typename DeriveClass>
class OpticsNodeType :public NodeTypeRegister<DeriveClass> {
protected:
	ref_ptr<OpticsNodeOutputData> oOutput;
	virtual void getAssistUI(NodeAssistUI& upstreamUI) override { 
		if(Node::isTurnOn())
			oOutput->data.displayOnViewport(upstreamUI);// for display element
	}
public:
	OpticsNodeType(int numberOfInput): NodeTypeRegister<DeriveClass>(numberOfInput) {
		oOutput = this->template addOutputData<OpticsNodeOutputData>();
	}
};

class OpticsSourceNode :public OpticsNodeType<OpticsSourceNode> {
protected:
	// variables
	enum SourceTypeEnum {
		PointSource = 0,
		ParallelSource,
	};
	int sourceType{ PointSource };

	int rayCount{ 10 };
	double wavelength{ 550 }; // in nm
	double apertureX{ 0 };
	double apertureSize{ 1 };

	Vec2 center{ -10,0 };

	double parallelAngle{ 0 }; // in degree
	double parallelStartOffset{ 10 };

public:
	static wstring getClassName() { return L"Optics Source"; }
	static vector<wstring> getCategoryName() { return { L"Element" }; }
	static ImageHandle getClassIcon() { return ImageHandle(WindowManager::resourceIconDir + L"OpticsSourceNode.jpg", true); }
	OpticsSourceNode() : OpticsNodeType<OpticsSourceNode>(1) {
		// setup parameters
		nodeParameter.addParams(L"Source Type", { L"Point Source", L"Parallel Source" }, &sourceType);
		nodeParameter.addParams(L"Ray Count", &rayCount, { 1,INT_MAX });
		nodeParameter.addParams(L"Wavelength (nm)", &wavelength, { 380,750 });
		nodeParameter.addParams(L"Aperture X", &apertureX);
		nodeParameter.addParams(L"Aperture Size", &apertureSize, {0,DBL_MAX});
		nodeParameter.addParams(L"Center", &center, {}, [&]() {return sourceType == PointSource; });
		nodeParameter.addParams(L"Angle", &parallelAngle, {}, [&]() {return sourceType == ParallelSource; });
		nodeParameter.addParams(L"Start Offset", &parallelStartOffset, {}, [&]() {return sourceType == ParallelSource; });
	}
	virtual bool bake()override;
};

class OpticsLensNode :public OpticsNodeType<OpticsLensNode> {
protected:
	// variables
	double positionX{ 0 };
	double curvatureRadius{ 10 };
	double refractiveIndex{ 1.5 };
	double dispersiveCoefficient{ 0 };
	bool isEntrance{ true };

public:
	static wstring getClassName() { return L"Optics Lens"; }
	static vector<wstring> getCategoryName() { return { L"Element" }; }
	static ImageHandle getClassIcon() { return ImageHandle(WindowManager::resourceIconDir + L"OpticsLensNode.jpg", true); }
	OpticsLensNode() : OpticsNodeType<OpticsLensNode>(1) {
		// setup parameters
		nodeParameter.addParams(L"Position X", &positionX);
		nodeParameter.addParams(L"Curvature Radius", &curvatureRadius);
		nodeParameter.addParams(L"Refractive Index", &refractiveIndex, { 1.0,DBL_MAX });
		nodeParameter.addParams(L"Dispersive Coefficient", &dispersiveCoefficient, { 0,DBL_MAX });
		nodeParameter.addParams(L"Is Entrance", &isEntrance);
	}
	virtual bool bake()override;
};

class RefractNode :public OpticsNodeType<RefractNode> {
public:
	static wstring getClassName() { return L"Optics Refract"; }
	static vector<wstring> getCategoryName() { return { L"Operation" }; }
	static ImageHandle getClassIcon() { return ImageHandle(WindowManager::resourceIconDir + L"OpticsRefractNode.jpg", true); }
	RefractNode() : OpticsNodeType<RefractNode>(2) {
	}
	virtual bool bake()override;
};
