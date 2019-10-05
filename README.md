Globe Puzzle: The Globe puzzle  is a spherical tile rotation puzzle similar to a Rubik’s Cube. The puzzle presents as a globe with three intersecting rings containing 12 tiles each. The rings are arraigned perpendicularly to one-another with one ring corresponding to the equator, and the other two corresponding to lines of longitude. The rings rotate freely with each tile occupying 30◦ of either latitude or longitude. In normal latitude and longitude the globe is divided into Noth and South Hemispheres with latitude being defined relative to the equator, while longitude is divided into East and West hemispheres relative to the prime meridian. I used a simpler notation where latitude coordinates run from 0◦ at the north pole to 180◦ at the south. Longitude goes for 360◦ around the globe. Thus in this coordinate system the vertical rings run along longitude 0◦ to 180◦ and 90◦ to 270◦, and the equator ring contains tiles at latitude 90◦. The vertical rings intersect at the ”North Pole” (latitude 0◦, longitude 0◦) and the ”South Pole” (latitude 180◦ and longitude 180◦). They intersect with the equator at the following points: (90◦, 0◦), (90◦, 180◦), (90◦, 90◦), and (90◦, 270◦).

Input: It is given in the format tileID "30-180" that is currently at latitude and longitude (90,270) and which has an exact target coordinates of latitude and longitude (30,180) to match. A puzzle is complete when all of its tiles are at their target locations.

The code used three algorithms for implementation of Search. 
1) Breadth First Search
2) AStar
3) Recursive Best First Search

The final result returned is the set of moves which needs to be done in sequence in order to solve the puzzle.
