#include "trojanmap.h"

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string &id)
{
  //xyx
  // int lat = data[id].lat;
  // return lat;

  // xcy
  if (data.find(id) == data.end())
    return -1;
  return data[id].lat;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist, return -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string &id)
{
  //xcy
  if (data.find(id) == data.end())
    return -1;
  return data[id].lon;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return "NULL".
 *
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string &id)
{
  //xcy
  if (data.find(id) == data.end())
    return "NULL";
  return data[id].name;
}
/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return an empty vector.
 *
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string &id)
{
  //xyx
  // std::vector<std::string> result;
  // result = data[id].neighbors;
  // return result;

  // xcy
  if (data.find(id) == data.end())
    return {};
  return data[id].neighbors;
}

/**
 * GetID: Given a location name, return the id.
 * If the node does not exist, return an empty string.
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 */
std::string TrojanMap::GetID(const std::string &name)
{
  //xyx
  // std::map<std::string, Node>::iterator iter;
  // for (iter = data.begin(); iter != data.end(); iter++)
  // {
  //   std::string nodename = iter->second.name;
  //   if (nodename.compare(name) == 0)
  //     return iter->second.id;
  // }

  // xcy
  for (auto &node : data)
  {
    if (node.second.name == name)
      return node.first;
  }
  return "";
}

/**
 * GetPosition: Given a location name, return the position. If id does not exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name)
{
  //xyx
  // std::map<std::string, Node>::iterator iter;
  // std::pair<double, double> results(-1, -1);
  //  for (iter = data.begin(); iter != data.end(); iter++)
  //  {
  //    std::string curname = iter->second.name;
  //    if (curname.compare(name) == 0)
  //    {
  //      results.first = iter->second.lat;
  //      results.second = iter->second.lon;
  //    }
  //  }

  // xcy
  std::pair<double, double> results(-1, -1);
  for (auto &node : data)
  {
    if (node.second.name == name)
      results = {node.second.lat, node.second.lon};
  }
  return results;
}

/**
 * CalculateEditDistance: Calculate edit distance between two location names
 *
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b)
{
  //xcy
  std::vector<std::vector<int>> res(a.length() + 1, std::vector<int>(b.length() + 1, 0));
  for (int i = 1; i <= a.length(); i++) {res[i][0] = i;}
  for (int j = 1; j <= b.length(); j++) {res[0][j] = j;}

  for (int i = 1; i <= a.length(); i++)
  {
    for (int j = 1; j <= b.length(); j++)
    {
      if (a[i - 1] == b[j - 1])
      {
        res[i][j] = std::min(res[i - 1][j], res[i][j - 1]);
        res[i][j] = std::min(res[i - 1][j - 1], res[i][j]);
      }
      else
      {
        res[i][j] = std::min(res[i - 1][j], res[i][j - 1]);
        res[i][j] = std::min(res[i - 1][j - 1], res[i][j]);
        res[i][j] += 1;
      }
    }
  }
  return res[a.length()][b.length()];
}

/**
 * FindClosestName: Given a location name, return the name with smallest edit distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : similar name
 */
std::string TrojanMap::FindClosestName(std::string name)
{
  //xcy
  int min = INT_MAX;
  std::string tmp;
  for(auto& node : data){
    if(CalculateEditDistance(name, node.second.name) < min){
      tmp = node.second.name;
      min = CalculateEditDistance(name, node.second.name);
    }
  }
  return tmp;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name)
{
  //xyx
  // int nsize = name.size();
  // std::map<std::string, Node>::iterator iter;
  // std::vector<std::string> results;
  // for (iter = data.begin(); iter != data.end(); iter++)
  // {
  //   std::string nodename = iter->second.name;
  //   if (nodename.size() < nsize)
  //     continue;
  //   std::string tmp = nodename.substr(0, nsize);
  //   std::string tmpname = name;
  //   std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::toupper);
  //   std::transform(tmpname.begin(), tmpname.end(), tmpname.begin(), ::toupper);
  //   if (tmpname.compare(tmp) == 0)
  //   {
  //     results.insert(results.end(), nodename);
  //   }
  // }
  // return results;

  // xcy
  std::vector<std::string> results;
  std::transform(name.begin(), name.end(), name.begin(),[](unsigned char c){return std::tolower(c);});

  for (auto &node : data)
  {
    std::string tmp = node.second.name;
    std::transform(tmp.begin(), tmp.end(), tmp.begin(),[](unsigned char c){ return std::tolower(c);});

    if (tmp.length() < name.length()) continue;
    if (tmp.compare(0, name.length(), name) == 0)
    {
      results.push_back(node.second.name);
    }
  }
  return results;
}

/**
 * CalculateDistance: Get the distance between 2 nodes.
 *
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id, const std::string &b_id)
{
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2), 2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2), 2.0);
  double c = 2 * asin(std::min(1.0, sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 *
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path)
{
  // Do not change this function
  double sum = 0;
  for (int i = 0; i < int(path.size()) - 1; i++)
  {
    sum += CalculateDistance(path[i], path[i + 1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name)
{
  /*yxy
  std::string source_id = GetID(location1_name);
  std::string destin_id = GetID(location2_name);
  std::stack<std::string> s;
  int nsize = data.size();
  // Create a index map, from id to vector index
  std::map<std::string, int> indexlist;
  std::map<std::string, Node>::iterator iter;
  int i;
  for (i = 0, iter = data.begin(); iter != data.end(); iter++, i++)
  {
    std::pair<std::string, int> newpair;
    newpair.first = iter->first;
    newpair.second = i;
    indexlist.insert(indexlist.end(), newpair);
  }
  // Create a visited distance vector and path frame
  std::vector<int> visited(nsize, 0);
  std::vector<double> distance(nsize, INT_MAX);
  std::vector<std::vector<std::string>> path(nsize);
  // initial
  distance[indexlist[source_id]] = 0;
  visited[indexlist[source_id]] = 1;
  path[indexlist[source_id]] = {source_id};
  int desindex = indexlist[destin_id];
  int srcindex = indexlist[source_id];
  s.push(source_id);
  // dijkstra algrithm
  while (visited[desindex] != 1)
  {
    std::string tmpsid = s.top();
    s.pop();
    Node tmpnode = data[tmpsid];
    int tmpnsize = tmpnode.neighbors.size();
    for (int i = 0; i < tmpnsize; i++)
    {
      std::string tmpnid = tmpnode.neighbors[i];
      if (visited[indexlist[tmpnid]] == 1)
        continue;
      double tmpdistance = CalculateDistance(data[tmpsid], data[tmpnid]);
      if (distance[indexlist[tmpsid]] + tmpdistance < distance[indexlist[tmpnid]])
      {
        distance[indexlist[tmpnid]] = distance[indexlist[tmpsid]] + tmpdistance;
        path[indexlist[tmpnid]].clear();
        path[indexlist[tmpnid]] = path[indexlist[tmpsid]];
        path[indexlist[tmpnid]].insert(path[indexlist[tmpnid]].end(), tmpnid);
      }
    }
    double mindis = INT_MAX;
    int minindex;
    for (int i = 0; i < nsize; i++)
    {
      if (visited[i] == 1)
        continue;
      if (distance[i] < mindis)
      {
        mindis = distance[i];
        minindex = i;
      }
    }
    visited[minindex] = 1;
    for (std::map<std::string, int>::iterator it2 = indexlist.begin(); it2 != indexlist.end(); it2++)
    {
      if (it2->second == minindex)
      {
        s.push(it2->first);
        break;
      }
    }
  }
  std::vector<std::string> x;
  x = path[desindex];
  return x;
  */
}
//xyx
// int fac(int i)
// {
//   if (i == 0)
//     return 1;
//   else if (i < 0)
//     return 1;
//   else
//     return i * fac(i - 1);
// }

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Do the early termination when there is no change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name)
{
  std::vector<std::string> path;
  return path;
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */

/* xyx
int fac(int i)
{
  if (i == 0)
    return 1;
  else if (i < 0)
    return 1;
  else
    return i * fac(i - 1);
}

int Permutation_DFS(std::vector<int> mark, int step, int all, std::vector<std::vector<int>> &path, std::vector<int> &curpath)
{
  if (step == all)
  {
    path.insert(path.end(), curpath);
    return 1;
  }
  int i;
  int total = 0;
  for (i = 1; i < all; i++)
  {
    if (mark[i])
      continue;
    mark[i] = 1;
    curpath[step] = i;
    total = total + Permutation_DFS(mark, step + 1, all, path, curpath);
    mark[i] = 0;
  }
  return total;
}

int isExist(std::vector<std::vector<int>> curpath, std::vector<int> allpath)
{
  int cpsize = curpath.size();
  return 0;
}

bool finishflag(std::vector<int> visited, std::vector<int> desindex)
{
  int dsize = desindex.size();
  for (int i = 0; i < dsize; i++)
  {
    if (visited[desindex[i]] == 1)
      continue;
    else
      return 0;
  }
  return 1;
}
*/
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(
    std::vector<std::string> location_ids)
{
  /* xyx
  std::pair<double, std::vector<std::vector<std::string>>> results;
  int lsize = location_ids.size();
  std::vector<std::vector<int>> path;
  int nsolu = fac(lsize - 1) / 2;
  // Create all solutions
  if (path.size() != nsolu)
  {
    std::vector<int> mark(lsize, 0);
    std::vector<std::vector<int>> tmppath;
    std::vector<int> curpath(lsize + 1, 0);
    Permutation_DFS(mark, 1, lsize, tmppath, curpath);
    int psize = tmppath.size();
    // delete reverse solution
    for (int i = 0; i < psize; i++)
    {
      if (isExist(path, tmppath[i]) == 1)
        continue;
      path.insert(path.end(), tmppath[i]);
    }
  }
  // Create distance map
  std::vector<std::vector<double>> shortesdis;
  std::vector<std::vector<std::vector<std::string>>> pathlocid;
  // for(int i=0;i<lsize;i++){
  //   pathlocid[i].resize(lsize);
  // }
  for (int i = 0; i < lsize; i++)
  {
    std::vector<double> shortdis(lsize);
    std::string source_id = location_ids[i];
    std::vector<std::string> destin_id = location_ids;
    std::vector<std::vector<std::string>> results = CalculateShortestDistance(source_id, destin_id, shortdis);
    pathlocid.insert(pathlocid.end(), results);
    shortesdis.insert(shortesdis.end(), shortdis);
  }
  // Create 3D vector
  // std::vector<std::vector<std::vector<std::string>>> pathlocid;
  int psize = path.size();
  std::vector<double> sumdis(psize, 0);
  std::vector<std::vector<std::string>> process(psize);
  for (int i = 0; i < psize; i++)
  {
    for (int j = 1; j < lsize + 1; j++)
    {
      sumdis[i] += shortesdis[j - 1][j];
    }
  }
  for (int i = 0; i < psize; i++)
  {
    for (int j = 0; j < lsize + 1; j++)
    {
      process[i].insert(process[i].end(), location_ids[path[i][j]]);
    }
  }
  int minindex = 0;
  for (int i = 1; i < psize; i++)
  {
    if (sumdis[minindex] > sumdis[i])
      minindex = i;
  }
  results.first = sumdis[minindex];
  results.second = process;
  location_ids = process[minindex];
  std::cout << results.second.size() << std::endl;
  return results;
  */
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
    std::vector<std::string> location_ids)
{
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
    std::vector<std::string> location_ids)
{
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename)
{
  std::vector<std::string> location_names_from_csv;
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename)
{
  std::vector<std::vector<std::string>> dependencies_from_csv;
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies. If there is no way to do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies)
{
  std::vector<std::string> result;
  return result;
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square)
{
  return false;
}

/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square)
{
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;
  return subgraph;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 *
 * @param {std::vector<std::string>} subgraph: list of location ids in the square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square)
{
  return false;
}

/**
 * FindNearby: Given a class name C, a location name L and a number r,
 * find all locations in class C on the map near L with the range of r and return a vector of string ids
 *
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {int} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k)
{
  std::vector<std::string> res;
  return res;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 *
 */
void TrojanMap::CreateGraphFromCSVFile()
{
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line))
  {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ','))
    {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else
      {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0]))
          n.attributes.insert(word);
        if (isdigit(word[0]))
          n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}
