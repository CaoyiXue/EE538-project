## Step 1: Autocomplete the location name

```c++
std::vector<std::string> Autocomplete(std::string name);
```
### Description:
1. Traverse all nodes on the map.
2. If the size of input is greater than the size of node’s name, we skip this node.
3. Otherwise, convert input name and node's name to lower cases. 
4. Then, compare input name and substring of node name from the beginning which have same length.
5. If they are equal, push the node name into result vector.

### Examples:
1. chi ==> Chinese Street Food, Chick-fil-A, Chipotle.\
Time taken by function: 4 ms

2. w ==> Washington & Broadway 1, Washington & Hope, Washington & Union 2, Washington & Hoover, Washington & Bonnie Brae, Washington & Union, Washington & Cherry, Washington & Figueroa 1,Washington & Figueroa, Washington & Vermont, Washington & Budlong, Washington & Normandie, Washington & Harvard, Washington & Westmoreland, Washington & Mariposa, Western & Exposition, Western & 36th, Western & Adams 3, Western & Adams 2, Western & Adams, Western & 24th 1, Western & 10 Off-ramp 1, Western & Washington, Western & 10 Off-ramp, Washington & Western, Washington & Arlington, Workshop Salon & Boutique, Warning Skate Shop, Western & 24th, Western & Jefferson 1, Washington & Olive, Western & Adams 1, Western & 29th 1, Wadsworth Elementary School, Western & Exposition 1, West Vernon Elementary School, Washington & Gramercy, Western & 29th, Which Wich?
, Washington 1, Washington & Flower, Washington, Washington Boulevard School, Western & 36th 1, Western Avenue Christian Church, West Adams Presbyterian Church, Ward African Methodist Episcopal Church, Westvern Station Los Angeles Post Office, Westminster Presbyterian Church, Washington & Union 1, Washington & Oak, Washington & Grand 1, Walker Temple African Methodist Episcopal Church, Washington & Grand, Washington & Oak 1, Washington & New England, West Side Church of God, Washington & Broadway, Western & Jefferson, Washington & Bonsallo\
Time taken by function: 5 ms

3. s ==> Stimson House, Saint Cecilia School, Smoke Shop, Stout Burger, St Agnes Church, Security Checkpoint, Shall Gas, Social Security Administration, SunLife Organics, Student Union (STU),Starbucks 3, San Pedro Street 2, Starbucks 2, Safety Pole, San Julian & 12th, Starbucks 1, Spiritual Fellowship Church, Studio 423, Soles Bicycles, San Pedro Street 1, Starbucks, St. Philips Episcopal Church, St. Johns Cathedral-Hope Net, Saint Lukes Missionary Baptist Church, San Pedro Street, Subway, Saint Agnes Elementary School, Saint James Park, Shell, Subway 1, Saint Marks Lutheran Church, Santa Barbara Avenue School, Sonny Astani School of Civil Engineering, Strangers Rest Baptist Church, Saint Patrick School, Sweet illusion, Second Baptist Church, Senshin Buddhist Church, Saint Patricks Catholic Church, Servants of Mary Convent, Saint John Baptist Church, Saint Phillips Episcopal Church, Schoenfeld Symphonic Hall, Safety Pole 1, St. Francis Center - Food Bank\
Time taken by function: 5 ms

### Observation:
Different size of inputs need roughly the same time because we need to traverse all nodes in the data. Time complexity is O(n), n is number of nodes. When we type in longer name, running time will decrease a little because we can skip more nodes with shorter name.

## Step 2-1: Find the place's Coordinates in the Map

```c++
std::pair<double, double> GetPosition(std::string name);
```

### Description:
1. Traverse all nodes on the map.
2. If the name of input is equal to one node’s name, return this node latitude and longitude.

### Examples:
1. 
```shell
Please input a location:Hill & Adams
*************************Results******************************
Latitude: 34.0262 Longitude: -118.27
**************************************************************
Time taken by function: 5 ms
```
2. 
```shell
Please input a location:Central Adult Senior High
*************************Results******************************
Latitude: 34.0338 Longitude: -118.267
**************************************************************
Time taken by function: 5 ms
```
3. 
```shell
Please input a location:Expo/Western 1
*************************Results******************************
Latitude: 34.0184 Longitude: -118.31
**************************************************************
Time taken by function: 3 ms
```

### Observation:
Time complexity in worst case should be O(n) in which n is number of nodes. 

## Step 2-2: Check edit distance between two location names

```c++
int CalculateEditDistance(std::string a, std::string b);
```
### Description:
1. Create vector "res" with shape (a.length+1, b.length+1). Initialize the base case, for example:\
a.length = 3, b.length = 2;\
    [0,1,2,3,4]\
    [1,0,0,0,0]\
    [2,0,0,0,0]
2. From i = 1, j = 1, using following equation to update:\
    if the i-1th character of a == the j-1th character of b, res[i][j] = min(res[i-1][j], res[i][j-1], res[i-1][j-1])\
    if the i-1th character of a != the j-1th character of b, res[i][j] = min(res[i-1][j], res[i][j-1], res[i-1][j-1])+1
3. Finally, return bottom-right corner element of res, which is res[a.length()][b.length()]

### Observation:
Time complexity is O((a.length+1) * (b.length+1)) and Memory size is (a.length+1) * (b.length+1)

```c++
std::string FindClosestName(std::string name);
```
### Description:
1. Initilize "min" as INT_MAX, "tmp" as empty string. Traverse all nodes on the map. 
2. If EditDistance between input name and node's name is less than "min", update min to current EditDistance and update tmp to current node's name.
3. Finally, return the ClosestName "tmp"

### Examples:
1. 
```shell
Please input a location:Hill & 11
*************************Results******************************
No matched locations.
Did you mean Hill & 11th instead of Hill & 11? [y/n]y
Latitude: 34.04 Longitude: -118.26
**************************************************************
Time taken by function: 5 ms
```
2. 
```shell
Please input a location:KF
*************************Results******************************
No matched locations.
Did you mean KFC instead of KF? [y/n]y
Latitude: 34.0261 Longitude: -118.278
**************************************************************
Time taken by function: 3 ms
```
3. 
```shell
Please input a location:Rolphs
*************************Results******************************
No matched locations.
Did you mean Ralphs instead of Rolphs? [y/n]y
Latitude: 34.0318 Longitude: -118.291
**************************************************************
Time taken by function: 5 ms
```

### Observation:
Assume maximum length of node's name is L, the number of nodes is n. In worst case, the running time is L*n, so the time complexity is O(n). Therefore, the function time when we type in wrong name is roughtly same as the function time when we type in right name.

## Step 3: CalculateShortestPath between two places

```c++
std::vector<std::string> CalculateShortestPath_Dijkstra(std::string &location1_name,
                                               std::string &location2_name);
```
### Description:
1. 
"res" store result of IDs\
"source" get start node ID\
"target" get end node ID\
"q" priority queue to update the minimum of distance,\
in the queue std::pair<double, std::string> make sure it will compare double variable first\
"marks" sets being visited node as 1, mark already visited node as 2\
"pre" updates previous node
```c++
std::vector<std::string> res; // store result of IDs
std::string source = GetID(location1_name); // get start node ID
std::string target = GetID(location2_name); // get target node ID
/* priority queue to update the minimum of distance, in the queue
std::pair<double, std::string> make sure it will compare double variable first
*/
std::priority_queue<std::pair<double, std::string>,
    std::vector<std::pair<double, std::string>>,
    std::greater<std::pair<double, std::string>>>q;
// mark being visited node as 1, mark already visited node as 2
std::map<std::string, int> marks;
// update previous node
std::map<std::string, std::string> pre;
```
2. 
create the distance map for initial case when none node is visited 
```c++
// create the distance map for initial case when none node is visited 
double infinite = std::numeric_limits<double>::max();
std::map<std::string, double> distances;
for (auto &node : data)
{
    distances[node.first] = infinite;
}
distances[source] = 0.0;
```
3. 
When q is not empty, 
pop out the minimum of distance in the queue, let already visited nodes in the queue pop out if they are minimum.\
mark being visited node as 1.\
traverse neighbors of being visited node\
if neighbor node is unvisited, calculate new distance from being visited node to this neighbor node\
compare the already stored distance with new distance, if new distance smaller, update distance and previous node of this neighbor node\
change being visited node to already visited node\
when target node is being visited, stop loop
```c++
q.push(std::make_pair(0.0, source));
while (!q.empty())
{
  std::string u = q.top().second;
  q.pop();//pop out the minimum of distance in the queue
  while (marks[u] == 2)
  {
 // let already visited nodes in the queue pop out if they are minimum
    u = q.top().second;
    q.pop();
  }
 // mark being visited node as 1
  marks[u] = 1;
  if (u == target)
  {
 //when target node is being visited stop loop
    break;
  }
  for (auto &child : data[u].neighbors)
  {// traverse neighbors of being visited node 
    if (marks[child] != 1 && marks[child] != 2)
    {// if neighbor node is unvisited, 
    // calculate new distance from being visited node to this neighbor node
      double alt = distances[u] + CalculateDistance(u, child);
      if (distances[child] > alt)
      {//compare the already stored distance with new distance
      // if new distance smaller, update distance and previous node of this neighbor node 
        distances[child] = alt;
        q.push(std::make_pair(alt, child));
        pre[child] = u;
      }
    }
  }
  //change being visited node to already visited node
  marks[u] = 2;
}
```
4. 
use pre map to backtrack the path from target node to start node\
reverse the result of IDs 
```c++
if (marks[target] == 1)
{//use pre map to backtrack the path from target node to start node
  res.push_back(target);
  while (target != source)
  {
    target = pre[target];
    res.push_back(target);
  }
  //reverse the result of IDs 
  std::reverse(res.begin(), res.end());
}
```
```c++
std::vector<std::string> CalculateShortestPath_Bellman_Ford(std::string &location1_name,
                                               std::string &location2_name);
```
### Description:
1. 
"path" store result of path\
"distance" update distances from source to all of other nodes\
"source" get start node ID\
"target" get target node ID\
"pre_path" update previous node\
"break_flag" flag to stop when find the target shortest path
```c++
  std::vector<std::string> path;// store result of path
  std::map<std::string, double> distance;// update distances from source to all of other nodes
  std::string source = GetID(location1_name);// get start node ID
  std::string target = GetID(location2_name);// get target node ID
  std::map<std::string, std::string> pre_path;// update previous node
  bool break_flag = false;// flag to stop when find the target shortest path
```
2. 
initialize distance map when we allow zero edge
```c++
  double infinite = std::numeric_limits<double>::max();
  std::map<std::string, std::vector<std::string>> predecessor = GetPredecessors();
  for (auto &node : predecessor)
  {// initialize distance map when we allow zero edge
    distance[node.first] = infinite;
  }
  distance[source] = 0.0;
```
3. 
let N is node number, from 0 to N-2, totally N-1 times\
traverse all node\
then traverse all predecessors of this node\
if distance from start to this predecessor is not infinite which means there is a path,\
then compare original distance with new distance from start to this node through this predecessor\
when distance from start to target is unchanged, stop inside loop\
set break_flag true to stop outside loop\
```c++
  for (int i = 0; i < predecessor.size() - 1; i++)
  { // let N is node number, from 0 to N-2, totally N-1 times
    double tmp = distance[target];
    for (auto &node : predecessor)
    {// traverse all node
      for (auto &p : node.second)
      {// then traverse all predecessors of this node
        if (distance[p] < infinite)
        {// if from start to this predecessor is not infinite which means there is a path, then compare original distance with new distance from start to this node through this predecessor 
          double asl = distance[p] + CalculateDistance(p, node.first);
          if (asl < distance[node.first])
          {
            distance[node.first] = asl;
            pre_path[node.first] = p;
          }
        }
      }
      if (tmp != infinite && node.first == target && tmp == distance[target])
      {// when distance from start to target is unchanged, stop inside loop
        break_flag = true;//set break_flag true to stop outside loop
        break; 
      }
    }
    if (break_flag)
    {
      break;
    }
  }
```
4. 
use pre_path map to backtrack the path from target node to start node\
reverse the result of IDs 
```c++
  if (distance[target] != infinite)
  {
    path.push_back(target);
    while (target != source)
    {
      target = pre_path[target];
      path.push_back(target);
    }
    std::reverse(path.begin(), path.end());
  }
```
### Examples:
```shell
Please input the start location:Ralphs
Please input the destination:Chick-fil-A
*************************Dijkstra*****************************
*************************Results******************************
"2578244375","4380040154","4380040153","4380040152","4380040148","6818427920","6818427919","6818427918","6818427892","6818427898","6818427917","6818427916","7232024780","6813416145","6813416154","6813416153","6813416152","6813416151","6813416155","6808069740","6816193785","6816193786","123152294","4015203136","4015203134","4015203133","21098539","6389467809","4015203132","3195897587","4015203129","4015203127","6352865690","6813379589","6813379483","3402887081","6814958394","3402887080","602606656","4872897515","4399697589","6814958391","123209598","6787673296","122728406","6807762271","4399697304","4399697302","5231967015","1862347583","3233702827","4540763379","6819179753","6820935900","6820935901","6813379556","6820935898","1781230450","1781230449","4015405542","4015405543","1837212104","1837212107","2753199985","6820935907","1837212100","4015372458","6813411588","1837212101","6814916516","6814916515","6820935910","4547476733",
The distance of the path is:1.49919 miles
**************************************************************
Time taken by function: 198 ms

*************************Bellman_Ford*************************
*************************Results******************************
"2578244375","4380040154","4380040153","4380040152","4380040148","6818427920","6818427919","6818427918","6818427892","6818427898","6818427917","6818427916","7232024780","6813416145","6813416154","6813416153","6813416152","6813416151","6813416155","6808069740","6816193785","6816193786","123152294","4015203136","4015203134","4015203133","21098539","6389467809","4015203132","3195897587","4015203129","4015203127","6352865690","6813379589","6813379483","3402887081","6814958394","3402887080","602606656","4872897515","4399697589","6814958391","123209598","6787673296","122728406","6807762271","4399697304","4399697302","5231967015","1862347583","3233702827","4540763379","6819179753","6820935900","6820935901","6813379556","6820935898","1781230450","1781230449","4015405542","4015405543","1837212104","1837212107","2753199985","6820935907","1837212100","4015372458","6813411588","1837212101","6814916516","6814916515","6820935910","4547476733",
The distance of the path is:1.49919 miles
**************************************************************
Time taken by function: 5769 ms
```
```shell
Please input the start location:Ralphs
Please input the destination:Target
*************************Dijkstra*****************************
*************************Results******************************
"2578244375","4380040154","4380040158","4380040167","6805802087","8410938469","6813416131","7645318201","6813416130","6813416129","123318563","452688940","6816193777","123408705","6816193774","452688933","452688931","123230412","6816193770","6787470576","4015442011","6816193692","6816193693","6816193694","4015377691","544693739","6816193696","6804883323","6807937309","6807937306","6816193698","4015377690","4015377689","122814447","6813416159","6813405266","4015372488","4015372487","6813405229","122719216","6813405232","4015372486","7071032399","4015372485","6813379479","6813379584","6814769289","5237417650",
The distance of the path is:0.927969 miles
**************************************************************
Time taken by function: 104 ms

*************************Bellman_Ford*************************
*************************Results******************************
"2578244375","4380040154","4380040158","4380040167","6805802087","8410938469","6813416131","7645318201","6813416130","6813416129","123318563","452688940","6816193777","123408705","6816193774","452688933","452688931","123230412","6816193770","6787470576","4015442011","6816193692","6816193693","6816193694","3398621886","3398621887","6816193695","5690152756","6804883324","3398621888","6813416123","6813416160","6818390184","6879730013","6879730020","6879730019","6818390183","6813405228","6813405229","6813405230","6813405231","6813405237","6813405235","6047197523","6813379584","6814769289","5237417650",
The distance of the path is:0.945321 miles
**************************************************************
Time taken by function: 1434 ms
```
```shell
Please input the start location:Exposition & Trousdale
Please input the destination:Bevvy
*************************Dijkstra*****************************
*************************Results******************************
"6474130386","6814958456","1862347576","4540690065","122728399","2700464020","2700466317","6814770333","4399697322","1771091166","6814955795","6814955797","6814955796","6814958393","3402887080","6814958394","3402887081","6813379483","6813379589","6352865690","4015203127","4015203129","3195897587","4015203132","6389467809","21098539","4015203133","4015203134","4015203136","123152294","6816193786","6816193785","6808069740","6813416155","6813416151","123152291","6813416144","123152289","6813416146","6813416142","7232024779","6807466006","7544351393","6813416140","6813416141","6787470555","6805743834","4015203141","6807909279","6818427897","122659220","6818427893","4380040147","6818427923","4015203145","4380040151","4015203146","8461002701","6813416135","6813416134","6813416133","8461002694","4015442014","8461002684","7265570419","7875057117","7760247600","6314934799","123408746","8749605858","4015442015","122613161","1841016361","8749605855","122613159","1771004847","1771004849","122935678","122921765","122440407","2607729726","6949934973","2607729722","2607729553","123250881","2607729747","123250884","2607729711","123454869","2607729745","123744935","2607729725","1717922465","4014925223","7396358386",
The distance of the path is:2.15752 miles
**************************************************************
Time taken by function: 351 ms

*************************Bellman_Ford*************************
*************************Results******************************
"6474130386","6814958456","1862347576","4540690065","122728399","2700464020","2700466317","6814770333","4399697322","1771091166","6814955795","6814955797","6814955796","6814958393","3402887080","6814958394","3402887081","6813379483","6813379589","6352865690","4015203127","4015203129","3195897587","4015203132","6389467809","21098539","4015203133","4015203134","4015203136","123152294","6816193786","6816193785","6808069740","6813416155","6813416151","123152291","6813416144","123152289","6813416146","6813416142","7232024779","6807466006","7544351393","6813416140","6813416141","6787470555","6805743834","4015203141","6807909279","6818427897","122659220","6818427893","4380040147","6818427923","4015203145","4380040151","4015203146","8461002701","6813416135","6813416134","6813416133","8461002694","4015442014","8461002684","7265570419","7875057117","7760247600","6314934799","123408746","8749605858","4015442015","122613161","1841016361","8749605855","122613159","1771004847","1771004849","122935678","122921765","122440407","2607729726","6949934973","2607729722","2607729553","123250881","2607729747","123250884","2607729711","123454869","2607729745","123744935","2607729725","1717922465","4014925223","7396358386",
The distance of the path is:2.15752 miles
**************************************************************
Time taken by function: 7003 ms
```

## Step 5: Cycle Detection

```c++
bool CycleDetection(std::vector<double> &square);
```
### Description:
1. 
inSquare function checks whether one node is in the square
```c++
bool inSquare(std::string id, std::vector<double> &square);
```
2. 
GetSubgraph function gets all of nodes in the square
```c++
std::vector<std::string> GetSubgraph(std::vector<double> &square);
```
3. 
CycleHelper function check whether there is a cycle in the part of graph which is considered as undirected graph based on DFS idea.
```c++
bool CycleHelper(std::string current_id, std::map<std::string, int> &marks,
                     std::string parent_id, std::vector<double> &square);
```
details:
```c++
  marks[current_id] = 1; // mark current node as 1
  for (const auto &child_id : data[current_id].neighbors)
  {// traverse all of children node of current node
    if (inSquare(child_id, square))
    {// we can only visit child nodes in the square
      if (marks[child_id] == 1 && child_id != parent_id)
      {// if child node is already visited which means already marked as 1 and it's not current node's parent node which is "parent_id", then cycle exists
        return true;
      }
      if (marks[child_id] != 1)
      {// if child node is not visited, then start recursive funtion to go deep
        if (CycleHelper(child_id, marks, current_id, square))
        {// if from child graph cycle exits, return true
          return true;
        }
      }
    }
  }
  // traverse all of nodes in this part of graph, return false
  return false;
```
4. 
```c++
bool CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square);
```
From the first node, use CycleHelper function to detect cycle.\
If there's not a cycle, use CycleHelper function to detect cycle from another unvisited node.\
If all nodes are already visited, return false.
```c++
  std::map<std::string, int> marks;
  for (const auto &node : subgraph)
  {
    if (marks[node] != 1)
    {
      if (CycleHelper(node, marks, "", square))
      {
        return true;
      }
    }
  }
  return false;
```
### Examples:
```shell
**************************************************************
* 5. Cycle Detection                                          
**************************************************************

Please input the left bound longitude(between -118.320 and -118.250):-118.26
Please input the right bound longitude(between -118.320 and -118.250):-118.275
Please input the upper bound latitude(between 34.000 and 34.040):34.023
Please input the lower bound latitude(between 34.000 and 34.040):34.011
*************************Results******************************
there exist no cycle in the subgraph 
**************************************************************
Time taken by function: 0 ms
```
```shell
**************************************************************
* 5. Cycle Detection                                          
**************************************************************

Please input the left bound longitude(between -118.320 and -118.250):-118.275
Please input the right bound longitude(between -118.320 and -118.250):-118.26
Please input the upper bound latitude(between 34.000 and 34.040):34.023
Please input the lower bound latitude(between 34.000 and 34.040):34.011
*************************Results******************************
there exists a cycle in the subgraph 
**************************************************************
Time taken by function: 0 ms
```

## Step 6: Topological Sort
```c++
std::vector<std::string> DeliveringTrojan(std::vector<std::string> &location_names,
                                            std::vector<std::vector<std::string>> &dependencies);
```
### Description:
1. 
From locations and dependencied, we can traverse them to adjacency list for delivering Trojan function. We need adjacency list for all of function so we add it as member variable.
```c++
std::unordered_map<std::string, std::vector<std::string>> AdjListDeliver;
```
2. 
Check circle exisitence for delivering Trojan function. Similar idea for step 5 CycleDetection, but here we detect cycle in directed graph
```c++
bool IsCycleDeliver_helper(std::string loc_name, std::map<std::string, int> &visited);
bool IsCycleDeliver();
```
3. 
Based on DFS idea, we can get the inverse of topological sort location names from part of graph.
```c++
void DeliverHelper(std::string loc_name,
        std::map<std::string, int> &marks, std::vector<std::string> &topo);
```
details:
```c++
  marks[loc_name] = 1;// mark current node as 1
  for (auto &child : AdjListDeliver[loc_name])
  {// traverse child nodes of this current node
    if (marks[child] != 1)
    {// if child node is unvisited, do recursive function, go deep
      DeliverHelper(child, marks, topo);
    }
  }
  // after traversing all nodes, push_back nodes from the bottom to the top.
  topo.push_back(loc_name);
```
4. 
```c++
std::vector<std::string> TrojanMap::DeliveringTrojan(
    std::vector<std::string> &locations,
    std::vector<std::vector<std::string>> &dependencies)
```
convert locations and dependencies to adjacency list which is stored in AdjListDeliver\
If has cycle in this map, return empty vector\
From the first node, get topological sort location names from part of graph.\
If there's unvisited node, continure to get topological sort location names startting from that node.\
If all nodes are already visited, reverse result vector and return.
```c++
  std::vector<std::string> result;// store result
  std::map<std::string, int> marks;// mark node whether visited

  for (auto &loc : locations)
  {// convert locations and dependencies to adjacency list
    std::vector<std::string> tmp;
    AdjListDeliver[loc] = tmp;
  }
  for (auto &dep : dependencies)
  {
    AdjListDeliver[dep[0]].push_back(dep[1]);
  }
  // If has cycle in this map, return empty vector
  if (IsCycleDeliver())
  {
    return result;
  }

  for (auto &loc : locations)
  {
    if (marks[loc] != 1)
    {
      DeliverHelper(loc, marks, result);
    }
  }

  std::reverse(result.begin(), result.end());
  return result;
```
