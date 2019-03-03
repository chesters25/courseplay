--[[
This file is part of Courseplay (https://github.com/Courseplay/courseplay)
Copyright (C) 2018 Peter Vajko

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
]]

---@class PathFinder
PathFinder = CpObject()
--- Generate nodes for the A* algorithm. The nodes
-- cover the polygon and are arranged in a grid.
--
function PathFinder:init()
	self.gridSpacing = 4
	self.count = 0
	self.yields = 0
	self.biasToRight = 0
	self.fruitToCheck = nil
	self.customHasFruitFunc = nil
end

function PathFinder:distance ( x1, y1, x2, y2 )
	return math.sqrt ( math.pow ( x2 - x1, 2 ) + math.pow ( y2 - y1, 2 ) )
end

function PathFinder:heuristicCostEstimate ( nodeA, nodeB )
	return self:distance ( nodeA.x, nodeA.y, nodeB.x, nodeB.y )
end

function PathFinder:lowestFScore ( set, f_score )
	local INF = 1/0
	local lowest, bestNode = INF, nil
	for _, node in ipairs ( set ) do
		local score = f_score [ node ]
		if score < lowest then
			lowest, bestNode = score, node
		end
	end
	return bestNode
end


function PathFinder:notIn( set, theNode )
	for _, node in ipairs ( set ) do
		if node == theNode then return false end
	end
	return true
end

function PathFinder:removeNode ( set, theNode )
	for i, node in ipairs ( set ) do
		if node == theNode then
			set [ i ] = set [ #set ]
			set [ #set ] = nil
			break
		end
	end
end

function PathFinder:unwindPath ( flat_path, map, current_node )
	if map [ current_node ] then
		table.insert ( flat_path, 1, map [ current_node ] )
		return self:unwindPath( flat_path, map, map [ current_node ] )
	else
		return flat_path
	end
end

function PathFinder:path ( start, goal, nodes, max_iterations )

	local closedset = {}
	local openset = { start }
	local came_from = {}
	local iterations = 0

	local g_score, f_score = {}, {}
	g_score [ start ] = 0
	f_score [ start ] = g_score [ start ] + self:heuristicCostEstimate ( start, goal )


	while #openset > 0 and iterations < max_iterations do
		iterations = iterations + 1
		local current = self:lowestFScore ( openset, f_score )
		if current == goal then
			local path = self:unwindPath ( {}, came_from, goal )
			table.insert ( path, goal )
			return path, iterations
		end

		self:removeNode ( openset, current )
		table.insert ( closedset, current )

		local neighbors = self:getNeighbors ( current, nodes )
		for _, neighbor in ipairs ( neighbors ) do
			if self:notIn ( closedset, neighbor ) then

				local tentative_g_score = g_score [ current ] + self:gScoreToNeighbor ( current, neighbor )

				if self:notIn ( openset, neighbor ) or tentative_g_score < g_score [ neighbor ] then
					came_from 	[ neighbor ] = current
					g_score 	[ neighbor ] = tentative_g_score
					f_score 	[ neighbor ] = g_score [ neighbor ] + self:heuristicCostEstimate ( neighbor, goal )
					if self:notIn ( openset, neighbor ) then
						table.insert ( openset, neighbor )
					end
				end
			end
		end
	end
	return nil, iterations -- no valid path
end


--
--- add some area with fruit for tests
function PathFinder:addFruitDistanceFromBoundary( grid, polygon )
	local distance = 10
	for y, row in ipairs( grid.map ) do
		for x, index in pairs( row ) do
			grid[index].isOnField = true
			local _, minDistanceToFieldBoundary = polygon:getClosestPointIndex({ x = grid[ index ].x, y = grid[ index ].y })
			if minDistanceToFieldBoundary > distance then
				grid[ index ].hasFruit = true
			elseif math.random(100) > 95 then
				grid[ index ].hasFruit = true
			end
		end
	end
end

function PathFinder:addFruitGridDistanceFromBoundary( grid, polygon )
	local distance = 1
	for y, row in ipairs( grid.map ) do
		for x, index in pairs( row ) do
			if x > distance + 1 and x <= #row - distance and y > distance and y <= #grid.map - distance  then
				grid[ index ].hasFruit = true
			end
		end
	end
end

function PathFinder:addIsland( grid, polygon )
	local distance = 64
	for y, row in ipairs( grid.map ) do
		for x, index in pairs( row ) do
			local _, minDistanceToFieldBoundary = polygon:getClosestPointIndex({ x = grid[ index ].x, y = grid[ index ].y })
			if minDistanceToFieldBoundary > distance then
				grid[ index ].isOnField = true
			end
		end
	end
end

--- Is this node an island (like a tree in the middle of the field)?
--
function PathFinder:isOnField( node )
	if courseGenerator.isRunningInGame() then
		if node.isOnField == nil then
			node.isOnField = courseplay:isField(node.x, - node.y)
		end
	else
		node.isOnField = true
	end
	return node.isOnField
end

--- Does the area around x, z has fruit?
--
function PathFinder:hasFruit(node, width)
	if self.customHasFruitFunc then
		return self.customHasFruitFunc(node, width)
	end
	if courseGenerator.isRunningInGame() then
		-- check the fruit if we haven't done so yet
		if node.hasFruit == nil then
			node.hasFruit = courseplay:areaHasFruit( node.x, -node.y, self.fruitToCheck, width )
		end
	end
	return node.hasFruit
end

function PathFinder:generateGridForPolygon( polygon, gridSpacingHint )
	local grid = {}
	-- map[ row ][ column ] maps the row/column address of the grid to a linear
	-- array index in the grid.
	grid.map = {}
	polygon.boundingBox = polygon:getBoundingBox()
	polygon:calculateData()
	-- this will make sure that the grid will have approximately 64^2 = 4096 points
	-- TODO: probably need to take the aspect ratio into accont for odd shaped
	-- (long and narrow) fields
	-- But don't go below a certain limit as that would drive too close to the fruite
	-- for this limit, use a fraction to reduce the chance of ending up right on the field edge (assuming fields
	-- are drawn using integer sizes) as that may result in a row or two missing in the grid
	self.gridSpacing = gridSpacingHint or math.max( 4.071, math.sqrt( polygon.area ) / 64 )
	local horizontalLines = generateParallelTracks( polygon, {}, self.gridSpacing, self.gridSpacing / 2 )
	if not horizontalLines then return grid end
	-- we'll need this when trying to find the array index from the
	-- grid coordinates. All of these lines are the same length and start
	-- at the same x
	grid.width = math.floor( horizontalLines[ 1 ].from.x / self.gridSpacing )
	grid.height = #horizontalLines
	-- now, add the grid points
	local margin = self.gridSpacing / 2
	for row, line in ipairs( horizontalLines ) do
		local column = 0
		grid.map[ row ] = {}
		for x = line.from.x, line.to.x, self.gridSpacing do
			column = column + 1
			for j = 1, #line.intersections, 2 do
				if line.intersections[ j + 1 ] then
					if x > line.intersections[ j ].x + margin and x < line.intersections[ j + 1 ].x - margin then
						local y = line.from.y
						-- check an area bigger than the self.gridSpacing to make sure the path is not too close to the fruit
						table.insert( grid, { x = x, y = y, column = column, row = row })
						grid.map[ row ][ column ] = #grid
					end
				end
			end
		end
	end
	return grid, self.gridSpacing
end

function PathFinder:findIslands( polygon )
	local grid, _ = self:generateGridForPolygon( polygon, Island.self.gridSpacing )
	local islandNodes = {}
	for _, row in ipairs( grid.map ) do
		for _, index in pairs( row ) do
			if not self:isOnField( grid[ index ]) then
				-- add a node only if it is far enough from the field boundary
				-- to filter false positives around the field boundary
				local _, d = polygon:getClosestPointIndex(grid[ index ])
				-- TODO: should calculate the closest distance to polygon edge, not
				-- the vertices. This may miss an island close enough to the field boundary
				if d > 8 * Island.self.gridSpacing then
					table.insert( islandNodes, grid[ index ])
					grid[ index ].island = true
				end
			end
		end
	end
	return islandNodes
end
--- Is 'node' a valid neighbor of 'theNode'?
--
function PathFinder:isValidNeighbor( theNode, node )
	-- this is called by a_star so we are in the x/y system
	--courseplay:debug( string.format( "theNode: %.2f, %2.f", theNode.x, theNode.y))
	--courseplay:debug( string.format( "node: %.2f, %.2f", node.x, node.y ))
	local d = self:distance( theNode.x, theNode.y, node.x, node.y )
	-- must be close enough (little more than sqrt(2) to allow for diagonals
	if d < self.gridSpacing * 1.5 then
		return true
	else
		return false
	end
end

--- a_star will call back here to get the valid neighbors
-- This is an optimization of the original algorithm which would iterate through all nodes
-- of the grid and see if theNode is close enough. We don't need that as we have our nodes in
-- a grid and we know exactly which (up to) eight nodes are the neighbors.
-- This reduces the iterations by two magnitudes
function PathFinder:getNeighbors( theNode, grid )
	local neighbors = {}
	self.count = self.count + 1
	if self.finder and self.count % 100 == 0 then
		self.yields = self.yields + 1
		coroutine.yield(false)
	end
	if theNode.column and theNode.row then
		-- we have the grid coordinates of theNode, we can figure out its neighbors
		-- how big is the area to check for neighbors?
		local width, height = 2, 2
		for column = theNode.column - width, theNode.column + width do
			for row = theNode.row - height, theNode.row + height do
				-- skip own node
				if not ( column == theNode.column and row == theNode.row ) and grid.map[ row ] and grid.map[ row ][ column ] then
					local neighbor = grid[ grid.map[ row ][ column ]]
					local theNodeIsOk = self:isOnField( theNode ) and not self:hasFruit( theNode, self.gridSpacing * 2 )
					local neighborIsOk = neighbor and self:isOnField( neighbor ) and not self:hasFruit( neighbor, self.gridSpacing * 2 )
					if neighbor and
						-- we only care about nodes with no fruit ...
						( neighborIsOk or
							-- ... or, if they have fruit, but the current node does not. This
							-- eliminates most nodes with fruit (except the ones close to the harvested area)
							-- and thus reduces the number of iterations significantly. However, no path will be
							-- generated when one of the end points is in the fruit.
							( not neighborIsOk and theNodeIsOk ))
					then
						table.insert( neighbors, neighbor )
						theNode.visited = true
					end
				end
			end
		end
	end
	if theNode.neighborIndexes then
		for _, nodeIndex in ipairs( theNode.neighborIndexes ) do
			table.insert( neighbors, grid[ nodeIndex ])
		end
	end
	return neighbors
end


--- g() score to neighbor, A star will call back here when calculating the score
--
function PathFinder:gScoreToNeighbor( node, neighbor )
	if self:hasFruit( neighbor, self.gridSpacing * 2 ) then
		-- this is the key parameter to tweak. This is basically the distance you are
		-- willing to travel in order not to cross one grid spacing of fruit. So, for
		-- example with a grid spacing of 3 meters, you rather go around 250 meters
		-- than to cross 3 meters of fruit. The purpose of this is to allow for a path
		-- with some fruit in it, which comes in handy when your combine is full on the
		-- first headland. This will minimize to amount of fruit you have to drive through.
		return 250
	else
		-- add a little bias so a path from point A to B will be slightly different from the
		-- point from B to A to avoid collisions when multiple tractors use the same route
		return self:distance( node.x + self.biasToRight, node.y + self.biasToRight, neighbor.x, neighbor.y )
	end
end

--- Add a non-grid node to the grid
-- the purpose of this is to set up the neighbors
function PathFinder:addOffGridNode( grid, newNode )
	for column, node in ipairs ( grid ) do
		if newNode ~= node and self:isValidNeighbor( newNode, node ) then
			if newNode.neighborIndexes then
				-- tell the new node about its neighbors
				table.insert ( newNode.neighborIndexes, column )
			else
				newNode.neighborIndexes = { column }
			end
			-- tell the other node about the new node
			if node.neighborIndexes then
				-- new node will be added as the last element of the grid
				table.insert( node.neighborIndexes, #grid + 1 )
			else
				node.neighborIndexes = { #grid + 1 }
			end
		end
	end
	table.insert( grid, newNode )
end


--- Find a path between from and to in a polygon using the A star
-- algorithm
-- expects x/y coordinates
function PathFinder:run( fromNode, toNode, polygon, fruit, customHasFruitFunc, addFruit, iterationsPerLoop )
	self.count = 0
	self.yields = 0
	self.fruitToCheck = fruit
	self.customHasFruitFunc = customHasFruitFunc
	local grid, width = self:generateGridForPolygon( polygon )
	-- hold a bit to the right
	self.biasToRight = fromNode.x < toNode.x and width / 2 or -width / 2
	if not courseGenerator.isRunningInGame() and addFruit then
		self:addFruitGridDistanceFromBoundary( grid, polygon )
	end
	courseGenerator.debug( "Grid generated with %d points", #grid)
	self:addOffGridNode( grid, fromNode )
	self:addOffGridNode( grid, toNode )
	-- limit number of iterations depending on the grid size to avoid long freezes
	local path = self:path( fromNode, toNode, grid, #grid)
	courseGenerator.debug( "Iterations %d, yields %d", self.count, self.yields)
	if path then
		path = Polyline:new( path )
		path:calculateData()
		path:smooth( math.rad( 0 ), math.rad( 180 ), 1 )
		courseGenerator.debug( "Path generated with %d points", #path )
		path:calculateData()
		path = space( path, math.rad( 15 ), 5 )
		courseGenerator.debug( "Path spaced, has now  %d points", #path )
		if not courseGenerator.isRunningInGame() then
			io.stdout:flush()
		end
	end
	return true, path, grid
end

function PathFinder:findPath(...)
	self:run(...)
end

function PathFinder:start(...)
	if not self.finder then
		self.finder = coroutine.create(self.run)
	end
	return self:resume(...)
end

function PathFinder:isActive()
	return self.finder ~= nil
end

function PathFinder:resume(...)
	local ok, done, path, grid = coroutine.resume(self.finder, self, ...)
	if not ok or done then
		self.finder = nil
		return false, path, grid
	end
	return true
end

---@class HeadlandPathFinder : PathFinder
HeadlandPathFinder = CpObject(PathFinder)

--- Default neighbor finder function, considering all nodes
function HeadlandPathFinder:getNeighbors( theNode, nodes )
	local neighbors = {}
	for _, node in ipairs ( nodes ) do
		if theNode ~= node and self:isValidNeighbor ( theNode, node ) then
			table.insert ( neighbors, node )
		end
	end
	return neighbors
end

-- g score does not need to take the fruit into account
function HeadlandPathFinder:gScoreToNeighbor(a, b)
	return self:distance( a.x, a.y, b.x, b.y )
end

--- Find path on the headland. This one currently ignores the fruit.
-- @param headlands - array of polygons containing the headland waypoints
-- #param dontUseInnermostHeadland - if true, won't use the innermost headland
function HeadlandPathFinder:findPath(fromNode, toNode, headlands, workWidth, dontUseInnermostHeadland)
	-- list of nodes for pathfinding are all the waypoints on the headland
	local nodes = {}
	local nHeadlandsToUse = math.max(1, dontUseInnermostHeadland and #headlands - 1 or #headlands)
	for i = 1, nHeadlandsToUse do
		for _, node in ipairs(headlands[i]) do
			table.insert(nodes, node)
		end
	end
	table.insert(nodes, fromNode)
	table.insert(nodes, toNode)

	courseGenerator.debug( "Starting pathfinding on headland (%d waypoints)", #nodes)

	-- this is for isValidNeighbor() to be able to find the closest points on the headland
	-- TODO: may need a customized isValidNeighbor if the working width is significantly smaller than the
	-- waypoint distance
	self.gridSpacing = math.max(courseGenerator.waypointDistance, workWidth) * 1.5
	local path, iterations = self:path(fromNode, toNode, nodes, #nodes * 3)
	courseGenerator.debug( "Number of iterations %d", iterations)
	if path then
		path = Polyline:new( path )
		path:calculateData()
		--path:smooth( math.rad( 0 ), math.rad( 180 ), 1 )
		courseGenerator.debug( "Path generated with %d points", #path )
	end
	return path, nodes
end

