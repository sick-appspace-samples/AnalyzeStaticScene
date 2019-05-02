--[[----------------------------------------------------------------------------
   
  Application Name:
  AnalyzeStaticScene
                                                                                    
  Summary:
  Segment static scene from a Visionary-T point cloud
   
  Description:
  This samples loads point cloud data from a file and
  - applies a RangeFilter
  - removes wall and table plane with the ShapeFitter
  - separates the objects on the table with Euclidean cluster extraction
   
  How to run:
  This sample can be run on the emulator or any device with
  AppEngine version 2.6.0 or higher and supporting PointClouds.
   
------------------------------------------------------------------------------]]
--Start of Global Scope---------------------------------------------------------

print('AppEngine version: ' .. Engine.getVersion())

-- Path to test file
local FILE_PATH = 'resources/static_scene.pcd'

-- Pausing duration for demonstration purpose only
local PAUSE = 2000

-- Set up viewer and decorations
local viewer = View.create()
viewer:setID('viewer3D')

local shapeDeco = View.ShapeDecoration.create()
shapeDeco:setFillColor(0, 0, 255, 20)

local pcDeco = View.PointCloudDecoration.create()
pcDeco:setPointSize(1)
pcDeco:setIntensityColormap(1)
viewer:setDefaultDecoration(pcDeco)

-- Set up ShapeFitter
local shapeFitter = PointCloud.ShapeFitter.create()
shapeFitter:setDistanceThreshold(30)
shapeFitter:setMaxIterations(500)
shapeFitter:setProbability(0.8)
shapeFitter:setOptimizeCoefficients(true)

--End of Global Scope-----------------------------------------------------------

--Start of Function and Event Scope---------------------------------------------

-- View point cloud and optional shape
local function viewPointCloud(pointCloud, shape, deco)
  viewer:clear()
  viewer:add(pointCloud)
  if nil ~= shape then
    viewer:addShape(shape, deco)
  end
  viewer:present()
  Script.sleep(PAUSE) -- For demonstration purpose only
end

-- Fit plane, show inliers, show outliers, return outlier cloud
--@fitAndRemovePlane(cloud:PointCloud):PointCloud
local function fitAndRemovePlane(cloud)
  -- Fit plane
  local tic = DateTime.getTimestamp()
  local plane, inliers = shapeFitter:fitPlane(cloud)
  local toc = DateTime.getTimestamp()
  print('Fitted plane in ' .. (toc - tic) .. ' ms.')
  local nx, ny, nz, distance = plane:getPlaneParameters()
  print( 'Plane parameters - nx: ' .. nx .. ' ny: ' .. ny .. ' nz: ' .. nz .. ' dist: ' .. distance )

  -- Extract inliers
  local inlierCloud = cloud:extractIndices(inliers)
  print('Extracted ' .. inlierCloud:getSize() .. ' inliers.')

  -- Visualize inliers and plane
  viewPointCloud(inlierCloud, plane, shapeDeco)

  -- Extract outliers
  local outlierCloud = cloud:extractIndices(inliers, true)
  print('Extracted ' .. outlierCloud:getSize() .. ' outliers.')

  -- Visualize extracted outliers
  viewPointCloud(outlierCloud)

  return outlierCloud
end

-- Entry point after Engine.OnStarted event
local function handleOnStarted()
  -- Load a point cloud from a file
  local cloud = PointCloud.load(FILE_PATH)

  -- Rotate the cloud for a better view (Visionary-T looks along the z-axis)
  local rotation = Transform.createRigidAxisAngle3D({0, 1, 0}, math.pi, 0, 0, 0)
  cloud = cloud:transform(rotation)

  -- View rotated cloud
  viewPointCloud(cloud)

  -- Remove edge hits with low intensity
  local range = PointCloud.RangeFilter.create()
  range:setIntensityRange(0.1, 1)
  cloud = range:filter(cloud)

  -- View filtered cloud
  viewPointCloud(cloud)

  -- Remove wall and table plane
  for i = 1, 2 do
    cloud = fitAndRemovePlane(cloud, shapeFitter, viewer)
  end

  -- Remove outliers
  range:setIntensityRange(0.15, 1)
  cloud = range:filter(cloud)
  cloud = cloud:filterRadiusOutliers(20, 5)

  -- View cloud
  viewPointCloud(cloud)

  -- Separate objects with Euclidean cluster extraction
  local clusters = cloud:segmentEuclideanClusters(20, 100)

  for i = 1, #clusters do
    local cluster = clusters[i]
    print('Object ' .. i .. ': ' .. PointCloud.getSize(cluster) .. ' points')
    -- View cluster
    viewPointCloud(cluster)
  end

  -- Select box (first object)
  local boxCloud = clusters[1]

  -- Get bounding box
  local box = boxCloud:getBoundingBox()
  print(box:toString())

  -- View box point cloud and bounding box.
  viewPointCloud(boxCloud, box, shapeDeco)

  print('App finished')
end
--The following registration is part of the global scope which runs once after startup
--Registration of the 'main' function to the 'Engine.OnStarted' event
Script.register('Engine.OnStarted', handleOnStarted)

--End of Function and Event Scope-----------------------------------------------
