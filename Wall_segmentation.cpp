#include <ogx/Plugins/EasyPlugin.h>
#include <ogx/Data/Clouds/CloudHelpers.h>
#include <ogx/Data/Clouds/SphericalSearchKernel.h>
#include <ogx/Data/Clouds/KNNSearchKernel.h>

using namespace ogx;
using namespace ogx::Data;
using namespace std;



struct Example : public ogx::Plugin::EasyMethod
{
	// parameters
	Data::ResourceID m_node_id;
	float radius = 7; //radius of local neighbourhood for segmentation


	// constructor
	Example() : EasyMethod(L"Marta Zawadzka", L"Algorithm aims to segment walls of buildings.")
	{
	}

	// add input/output parameters
	virtual void DefineParameters(ParameterBank& bank)
	{
		bank.Add(L"node_id", m_node_id).AsNode();
		bank.Add(L"Neighborhood radius", radius);
	}


	void SmoothCloud(ogx::Data::Clouds::ICloud* cloud, Context& context)
	{
		int progress = 0;

		//Get all points from the cloud
		ogx::Data::Clouds::PointsRange pointsRange;
		cloud->GetAccess().GetAllPoints(pointsRange);

		//Container for neighbourhood
		ogx::Data::Clouds::PointsRange KNNRange;

		//Coordinates container
		std::vector<ogx::Data::Clouds::Point3D> coordinates;
		coordinates.reserve(pointsRange.size());

		//Variables in for loop
		ogx::Count neighbours = 6;
		for (auto& xyz : ogx::Data::Clouds::RangeLocalXYZ(pointsRange)) {

			auto searchKernel = ogx::Data::Clouds::KNNSearchKernel(xyz.cast<double>(), neighbours);
			KNNRange.clear();
			//Calculating neighbourhood
			cloud->GetAccess().FindPoints(searchKernel, KNNRange);

			//Smoothing
			ogx::Data::Clouds::Point3D point = ogx::Data::Clouds::Point3D::Zero();
			for (const auto& pointAverage : ogx::Data::Clouds::RangeLocalXYZConst(KNNRange)) {
				point += pointAverage;
			}
			point = point / (double)KNNRange.size();
			coordinates.emplace_back(point);

			//Create progress bar
			progress++;
			if (!context.Feedback().Update((float)progress / (float)pointsRange.size()))
				ReportError(L"Could not update progress");
		}
		//Save calculated information
		pointsRange.SetXYZ(coordinates);

		OGX_LINE.Msg(ogx::Level::Info, L"Point cloud is now smoothed.");
	}

	void CalculateNormals(ogx::Data::Clouds::ICloud* cloud, Context& context) 
	{
		int progress = 0;

		//Get all points from cloud
		ogx::Data::Clouds::PointsRange pointsRange;
		cloud->GetAccess().GetAllPoints(pointsRange);

		//Neighbours
		ogx::Data::Clouds::PointsRange neighboursRange;
		const int neighboursAmount = 10;
		auto searchKNNKernel = ogx::Data::Clouds::KNNSearchKernel(ogx::Math::Point3D(0,0,0), neighboursAmount);

		//Normals
		std::vector<ogx::Data::Clouds::Point3D> normals;
		normals.reserve(pointsRange.size());

		for (auto& xyz : ogx::Data::Clouds::RangeLocalXYZConst(pointsRange)) {

			searchKNNKernel.GetPoint() = xyz.cast<double>();
			ogx::Data::Clouds::PointsRange neighboursRange;
			cloud->GetAccess().FindPoints(searchKNNKernel, neighboursRange);
			auto neighboursXYZ = ogx::Data::Clouds::RangeLocalXYZConst(neighboursRange);
			auto plane = ogx::Math::CalcBestPlane3D(neighboursXYZ.begin(), neighboursXYZ.end());
			ogx::Math::Point3D normal = plane.normal();
			normals.emplace_back(plane.normal().cast<float>());

			//Create progress bar
			progress++;
			if (!context.Feedback().Update((float)progress / (float)pointsRange.size()))
				ReportError(L"Could not update progress");
		}
		pointsRange.SetNormals(normals);

		OGX_LINE.Msg(ogx::Level::Info, L"Normal vectors calculated successfully.");
	}

	void Angles(ogx::Data::Clouds::ICloud* cloud, Context& context)
	{
		int progress = 0;
		double angle;

		//Get all points from the cloud
		ogx::Data::Clouds::PointsRange pointsRange;
		cloud->GetAccess().GetAllPoints(pointsRange);

		//Normals
		std::vector<ogx::Data::Clouds::Point3D> normals;
		pointsRange.GetNormals(normals);

		//Vector to calculate angle (plane XY normal)
		ogx::Math::Vector3D referenceVector = { 0,0,1 };

		int i = 0;
		//Stuff for adding layer
		std::vector<StoredReal> angleValues;
		angleValues.reserve(pointsRange.size());

		for (auto& xyz : ogx::Data::Clouds::RangeLocalXYZConst(pointsRange)) {

			//Calculate and save angles
			ogx::Math::Vector3D temporaryVector = normals[i].cast<double>();
			angle = ogx::Math::CalcAngleBetweenTwoVectors(referenceVector, temporaryVector);
			//Radians to degrees
			angle = (angle * 360) / (2 * PI);
			if (angle > 90) angle = 180 - angle;
			angle = 90 - angle;
			angleValues.emplace_back(angle);
			i++;
			//Create progress bar
			progress++;
			if (!context.Feedback().Update((float)progress / (float)pointsRange.size()))
				ReportError(L"Could not update progress");
		}
		//Saving calculated information to layer
		SaveInformationToLayer(cloud, angleValues, L"Horizontal Angles");
		
		OGX_LINE.Msg(ogx::Level::Info, L"Angles are calculated.");
	}

	void SmoothAngles(ogx::Data::Clouds::ICloud* cloud, Context& context)
	{
		int progress = 0;

		//Get all points from the cloud
		ogx::Data::Clouds::PointsRange pointsRange;
		cloud->GetAccess().GetAllPoints(pointsRange);

		//Container for neighbourhood
		ogx::Data::Clouds::PointsRange KNNRange;

		//Container for smoothed angles
		std::vector<ogx::StoredReal> smoothedAngles;
		smoothedAngles.reserve(pointsRange.size());

		//Access to layer values
		const ogx::String layerName = L"Horizontal Angles";
		auto layers = cloud->FindLayers(layerName);
		if (layers.size() != 1)
			ReportError(std::to_wstring(layers.size()) + L" layers found instead of 1!");
		auto layer = layers[0];
		if (layer == nullptr)
			ReportError(L" Failed to read new layer with name: " + layerName + L"!");
		std::vector<ogx::StoredReal> layerValues;
		pointsRange.GetLayerVals(layerValues, *layer);
		auto layerValuesIterator = layerValues.begin();


		//Variables in for loop
		ogx::Count neighbours = 60;
		ogx::Math::Vector3D referenceVector = { 0,0,1 }; //Z-axis
		for (auto& xyz : ogx::Data::Clouds::RangeLocalXYZConst(pointsRange)) {

			auto searchKernel = ogx::Data::Clouds::KNNSearchKernel(xyz.cast<double>(), neighbours);
			KNNRange.clear();
			//Calculating neighbourhood
			cloud->GetAccess().FindPoints(searchKernel, KNNRange);

			//Smoothing
			ogx::Data::Clouds::Point3D normal = ogx::Data::Clouds::Point3D::Zero();
			for (const auto& normalAverage : ogx::Data::Clouds::RangeLocalNormalConst(KNNRange)) {
				normal += normalAverage;
			}
			normal = normal / (double)KNNRange.size();
			StoredReal angle = ogx::Math::CalcAngleBetweenTwoVectors(referenceVector, normal.cast<double>());

			//Radians to degrees
			angle = (angle * 360) / (2 * PI);
			if (angle > 90) angle = 180 - angle;
			angle = 90 - angle;
			angle = (angle + layerValues[progress]) / 2;
			smoothedAngles.emplace_back(angle);

			//Create progress bar
			progress++;
			if (!context.Feedback().Update((float)progress / (float)pointsRange.size()))
				ReportError(L"Could not update progress");
		}

		//Saving calculated information to layer
		SaveInformationToLayer(cloud, smoothedAngles, L"Smoothed Angles");
		OGX_LINE.Msg(ogx::Level::Info, L"Angles are now smoothed.");
	}
	
	void SetInitialValues(ogx::Data::Clouds::ICloud* cloud, Context& context, const float angleThreshold=30.0)
	{
		int progress = 0;

		//Get all points from the cloud
		ogx::Data::Clouds::PointsRange pointsRange;
		cloud->GetAccess().GetAllPoints(pointsRange);

		//Container to store data (needed to add layer)
		std::vector<StoredReal> indices;
		indices.reserve(pointsRange.size());

		//Access to layer values
		const ogx::String layerName = L"Smoothed Angles";
		auto layers = cloud->FindLayers(layerName);
		if (layers.size() != 1)
			ReportError(std::to_wstring(layers.size()) + L" layers found instead of 1!");
		auto layer = layers[0];
		if (layer == nullptr)
			ReportError(L" Failed to read new layer with name: " + layerName + L"!");
		std::vector<ogx::StoredReal> layerValues;
		pointsRange.GetLayerVals(layerValues, *layer);
		auto layerValuesIterator = layerValues.begin();

		//Variables in for loop
		int wallIndex = 0;
		int defaultWallIndex = 0;
		for (auto& xyz : ogx::Data::Clouds::RangeLocalXYZConst(pointsRange)) {

			if (layerValues[progress] > angleThreshold) {
				indices.emplace_back(defaultWallIndex);
			}
			else if (layerValues[progress] <= angleThreshold) {
				wallIndex++;
				indices.emplace_back(wallIndex);
			}

			//Create progress bar
			progress++;
			if (!context.Feedback().Update((float)progress / (float)pointsRange.size()))
				ReportError(L"Could not update progress");
		}
		//Saving calculated information to layer
		SaveInformationToLayer(cloud, indices, L"Wall Index");
		OGX_LINE.Msg(ogx::Level::Info, L"Initial values are set.");
	}

	void SaveInformationToLayer(ogx::Data::Clouds::ICloud* cloud, std::vector<StoredReal> values, const ogx::String &layerName)
	{
		//Get all points from the cloud
		ogx::Data::Clouds::PointsRange pointsRange;
		cloud->GetAccess().GetAllPoints(pointsRange);

		//Check if layer for values exists
		auto const layers = cloud->FindLayers(layerName);

		// We can have many layers with the same name, take the first one
		// or create new if none found
		auto valuesLayer = layers.empty() ? cloud->CreateLayer(layerName, 0.0) : layers[0];

		//Save buffer with values
		pointsRange.SetLayerVals(values, *valuesLayer);
	}
	
	void SegmentWalls(ogx::Data::Clouds::ICloud* cloud, Context& context)
	{
		//Get all points from the cloud
		ogx::Data::Clouds::PointsRange pointsRange;
		cloud->GetAccess().GetAllPoints(pointsRange);

		//Container for neighbourhood
		ogx::Data::Clouds::PointsRange sphericalRange;

		//Access to layer values
		const ogx::String layerName = L"Wall Index";
		auto layers = cloud->FindLayers(layerName);
		if (layers.size() != 1)
			ReportError(std::to_wstring(layers.size()) + L" layers found instead of 1!");
		auto layer = layers[0];
		if (layer == nullptr)
			ReportError(L" Failed to read new layer with name: " + layerName + L"!");
		std::vector<ogx::StoredReal> layerValues;
		pointsRange.GetLayerVals(layerValues, *layer);
		auto layerValuesIterator = layerValues.begin();

		//Variables in for loop
		std::vector<ogx::StoredReal> temporaryNeighborsIndices;
		int i = 0;
		int minIndex = 1;
		int rightIndex = pointsRange.size();
		bool hasNonZeroNeighbour = false;
		for (const auto& xyz : ogx::Data::Clouds::RangeLocalXYZConst(pointsRange)) {

			if (layerValues[i] != 0) {
				auto searchKernel = ogx::Data::Clouds::SphericalSearchKernel(ogx::Math::Sphere3D(radius, xyz.cast<double>()));
				sphericalRange.clear();
				cloud->GetAccess().FindPoints(searchKernel, sphericalRange); //calculating neighbourhood
				temporaryNeighborsIndices.clear();
				sphericalRange.GetLayerVals(temporaryNeighborsIndices, *layer);//!
				for (const auto& idx : temporaryNeighborsIndices) {
					if (idx != 0) {
						hasNonZeroNeighbour = true;
						if (idx < minIndex) {
							minIndex = idx;
						}
					}
				}
				if (hasNonZeroNeighbour) {
					if (layerValues[i] > minIndex) {
						layerValues[i] = minIndex;
					}
				}
				else layerValues[i] = 0;
			}
			minIndex = rightIndex;
			i++;
			hasNonZeroNeighbour = false;
		}
		//Save calculated information to layer (this layer already exists)
		SaveInformationToLayer(cloud, layerValues, L"Wall Index");
	}

	virtual void Run(Context& context)
	{
		OGX_LINE.Msg(ogx::Level::Info, L"Starting plugin...");

		auto node = context.m_project->TransTreeFindNode(m_node_id);
		if (!node)
			ReportError(L"NodeID not found!");
		
		auto element = node->GetElement();
		if (!element)
			ReportError(L"Element not found");
		

		auto cloud = element->GetData<ogx::Data::Clouds::ICloud>();
		if (!cloud)
			ReportError(L"Cloud not found");
		
		int algorithmCount = 0;
		const int algorithmsAmount = 6;

		//Get all points from the cloud
		ogx::Data::Clouds::PointsRange pointsRange;
		cloud->GetAccess().GetAllPoints(pointsRange);

		OGX_LINE.Msg(ogx::Level::Info, L"Smoothing cloud..." + to_wstring(++algorithmCount) + L"/" + to_wstring(algorithmsAmount));
		SmoothCloud(cloud, context);

		OGX_LINE.Msg(ogx::Level::Info, L"Calculating normal vectors..." + to_wstring(++algorithmCount) + L"/" + to_wstring(algorithmsAmount));
		CalculateNormals(cloud, context);

		OGX_LINE.Msg(ogx::Level::Info, L"Calculating horizontal angles..." + to_wstring(++algorithmCount) + L"/" + to_wstring(algorithmsAmount));
		Angles(cloud, context);

		OGX_LINE.Msg(ogx::Level::Info, L"Smoothing horizontal angles..." + to_wstring(++algorithmCount) + L"/" + to_wstring(algorithmsAmount));
		SmoothAngles(cloud, context);

		OGX_LINE.Msg(ogx::Level::Info, L"Calculating Initial Values..." + to_wstring(++algorithmCount) + L"/" + to_wstring(algorithmsAmount));
		SetInitialValues(cloud, context);

		
		OGX_LINE.Msg(ogx::Level::Info, L"Wall segmentation in progress..." + to_wstring(++algorithmCount) + L"/" + to_wstring(algorithmsAmount));
		
		//Variables in for loop
		int segmentationProgress = 0;
		int iterations = 10;

		//We call function for segmentation a few times to get fine results
		for (int i = 0; i < iterations; i++) {

			SegmentWalls(cloud, context);

			//Create progress bar
			segmentationProgress++;
			if (!context.Feedback().Update((float)segmentationProgress / (float)iterations))
				ReportError(L"Could not update progress");
		}
		OGX_LINE.Msg(ogx::Level::Info, L"Wall segmentation is done.");

		OGX_LINE.Msg(ogx::Level::Info, L"I've just finished! :)");

	}
	
};

OGX_EXPORT_METHOD(Example)

