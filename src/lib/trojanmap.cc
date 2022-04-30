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
  std::pair<double, double> results(-1, -1);
  for (auto &node : data)
  {
    if (node.second.name == name)
      results = {node.second.lat, node.second.lon};
  }
  return results;
}

/**
 * CalculateEditDistance without dynamic programming
 */
int TrojanMap::EDHelper(std::string &a, std::string &b, int m, int n)
{
  if (m == 0)
    return n;
  if (n == 0)
    return m;

  if (a[m - 1] == b[n - 1])
  {
    return EDHelper(a, b, m - 1, n - 1);
  }

  return 1 + std::min(std::min(EDHelper(a, b, m, n - 1), EDHelper(a, b, m - 1, n)), EDHelper(a, b, m - 1, n - 1));
}

int TrojanMap::CalculateEditDistance_noDP(std::string a, std::string b)
{
  return EDHelper(a, b, a.length(), b.length());
}

std::string TrojanMap::FindClosestName_noDP(std::string name)
{
  int min = INT_MAX;
  std::string res;
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c)
                 { return std::tolower(c); });
  for (auto &node : data)
  {
    std::string tmp = node.second.name;
    if (tmp != "")
    {
      std::transform(tmp.begin(), tmp.end(), tmp.begin(), [](unsigned char c)
                     { return std::tolower(c); });
      int next = CalculateEditDistance_noDP(name, tmp);
      if (next < min)
      {
        res = node.second.name;
        min = next;
      }
    }
  }
  return res;
}

/**
 * CalculateEditDistance: Calculate edit distance between two location names
 *
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b)
{
  std::vector<std::vector<int>> res(a.length() + 1, std::vector<int>(b.length() + 1, 0));
  for (int i = 1; i <= a.length(); i++)
  {
    res[i][0] = i;
  }
  for (int j = 1; j <= b.length(); j++)
  {
    res[0][j] = j;
  }

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
  int min = INT_MAX;
  std::string res;
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c)
                 { return std::tolower(c); });
  for (auto &node : data)
  {
    std::string tmp = node.second.name;
    if (tmp != "")
    {
      std::transform(tmp.begin(), tmp.end(), tmp.begin(), [](unsigned char c)
                     { return std::tolower(c); });
      int next = CalculateEditDistance(name, tmp);
      if (next < min)
      {
        res = node.second.name;
        min = next;
      }
    }
  }
  return res;
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
  std::vector<std::string> results;
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c)
                 { return std::tolower(c); });

  for (auto &node : data)
  {
    std::string tmp = node.second.name;
    std::transform(tmp.begin(), tmp.end(), tmp.begin(), [](unsigned char c)
                   { return std::tolower(c); });

    if (tmp.length() < name.length())
      continue;
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
  std::vector<std::string> res;
  std::string source = GetID(location1_name);
  std::string target = GetID(location2_name);
  if (location1_name.empty() || location2_name.empty() || source.empty() || target.empty())
  {
    return res;
  }
  std::priority_queue<std::pair<double, std::string>,
                      std::vector<std::pair<double, std::string>>,
                      std::greater<std::pair<double, std::string>>> q;
  std::map<std::string, double> distances;
  std::map<std::string, int> marks;
  std::map<std::string, std::string> pre;
  distances[source] = 0.0;
  q.push(std::make_pair(0.0, source));
  while (!q.empty())
  {
    std::string u = q.top().second;
    q.pop();
    while (marks[u] == 2)
    {
      u = q.top().second;
      q.pop();
    }
    marks[u] = 1;

    if (u == target)
    {
      break;
    }

    for (auto &child : data[u].neighbors)
    {
      if (marks.find(child) == marks.end())
      {
        if (distances.find(child) == distances.end())
        {
          double alt = distances[u] + CalculateDistance(u, child);
          distances[child] = alt;
          q.push(std::make_pair(alt, child));
          pre[child] = u;
        }
        else
        {
          double alt = distances[u] + CalculateDistance(u, child);
          if (distances[child] > alt)
          {
            distances[child] = alt;
            q.push(std::make_pair(alt, child));
            pre[child] = u;
          }
        }
      }
    }
    marks[u] = 2;
  }

  if (marks[target] == 1)
  {
    res.push_back(target);
    while (target != source)
    {
      target = pre[target];
      res.push_back(target);
    }
    std::reverse(res.begin(), res.end());
  }

  return res;
}

/**
 * Map of node to its predecessors function for BellmanFord shortest path
 */
std::map<std::string, std::vector<std::string>> TrojanMap::GetPredecessors()
{
  std::map<std::string, std::vector<std::string>> pres;
  for (auto &node : data)
  {
    for (auto &child : node.second.neighbors)
    {
      pres[child].push_back(node.first);
    }
  }
  return pres;
}

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
  std::map<std::string, double> distance;
  std::string source = GetID(location1_name);
  std::string target = GetID(location2_name);
  if (location1_name.empty() || location2_name.empty() || source.empty() || target.empty())
  {
    return path;
  }

  std::map<std::string, std::string> pre_path;

  double infinite = std::numeric_limits<double>::max();
  std::map<std::string, std::vector<std::string>> predecessor = GetPredecessors();
  int count = 0;
  for (auto &node : predecessor)
  {
    distance[node.first] = infinite;
  }
  distance[source] = 0.0;
  for (int i = 0; i < predecessor.size() - 1; i++)
  {
    double tmp = distance[target];
    for (auto &node : predecessor)
    {
      for (auto &p : node.second)
      {
        if (distance[p] < infinite)
        {
          double asl = distance[p] + CalculateDistance(p, node.first);
          if (asl < distance[node.first])
          {
            distance[node.first] = asl;
            pre_path[node.first] = p;
          }
        }
      }
      if (tmp != infinite && node.first == target && tmp == distance[target])
      {
        count++;
      }
    }
    if (count > 9)
    {
      break;
    }
  }
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
  return path;
}

/**
 * Travelling salesman Brute Force & Backtracking Helper
 */
void TrojanMap::TravellingTrojan_helper(
    int cur_node, double cur_cost, std::vector<std::string> &cur_path,
    std::pair<double, std::vector<std::vector<std::string>>> &records,
    std::vector<std::string> &location_ids, bool &isBacktracking)
{
  if (cur_path.size() == location_ids.size())
  {
    double final_cost = cur_cost + CalculateDistance(location_ids[cur_node], location_ids[0]);
    if (final_cost < records.first)
    {
      records.first = final_cost;
      cur_path.push_back(location_ids[0]);
      records.second.push_back(cur_path);
      cur_path.pop_back();
    }
    return;
  }
  if (isBacktracking)
  {
    if (cur_cost >= records.first)
    {
      return;
    }
  }

  for (int i = 1; i < location_ids.size(); i++)
  {
    if (std::find(cur_path.begin(), cur_path.end(), location_ids[i]) != cur_path.end())
    {
      continue;
    }
    cur_path.push_back(location_ids[i]);
    TravellingTrojan_helper(i, cur_cost + CalculateDistance(location_ids[cur_node], location_ids[i]), cur_path, records, location_ids, isBacktracking);
    cur_path.pop_back();
  }
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(
    std::vector<std::string> location_ids)
{
  std::pair<double, std::vector<std::vector<std::string>>> records;
  if (location_ids.empty())
    return records;
  records.first = std::numeric_limits<double>::max();
  std::vector<std::string> cur_path{location_ids[0]};
  bool isBacktracking = false;
  TravellingTrojan_helper(0, 0.0, cur_path, records, location_ids, isBacktracking);
  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
    std::vector<std::string> location_ids)
{
  std::pair<double, std::vector<std::vector<std::string>>> records;
  if (location_ids.empty())
    return records;
  records.first = std::numeric_limits<double>::max();
  std::vector<std::string> cur_path{location_ids[0]};
  bool isBacktracking = true;
  TravellingTrojan_helper(0, 0.0, cur_path, records, location_ids, isBacktracking);
  return records;
}
/**
 * TwoOptSwap for TravellingTrojan_2opt
 */
std::vector<std::string> TrojanMap::TwoOptSwap(int i, int k, std::vector<std::string> location_ids)
{
  std::reverse(location_ids.begin() + i, location_ids.begin() + k + 1);
  return location_ids;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
    std::vector<std::string> location_ids)
{
  std::pair<double, std::vector<std::vector<std::string>>> records;
  if (location_ids.empty())
    return records;
  
  std::string source = location_ids[0];
  location_ids.push_back(source);
  records.first = CalculatePathLength(location_ids);
  records.second.push_back(location_ids);
  bool breakflag = false;
  int size = location_ids.size();
  int unchanged = 0;

  while (unchanged < 60)
  {
    unchanged++;
    breakflag = false;
    for (int i = 0; i < size - 2; i++)
    {
      for (int k = i + 1; k < size - 1; k++)
      {
        std::vector<std::string> cur_ids = records.second.back();
        cur_ids.pop_back();
        std::vector<std::string> new_ids = TwoOptSwap(i, k, cur_ids);
        new_ids = correct_order(new_ids, source);
        new_ids.push_back(source);
        double new_distance = CalculatePathLength(new_ids);
        if (new_distance < records.first)
        {
          unchanged = 0;
          records.first = new_distance;
          records.second.push_back(new_ids);
          breakflag = true;
          break;
        }
      }
      if (breakflag)
        break;
    }
  }
  return records;
}

/**
 * 3opt
 */
std::vector<std::vector<int>> TrojanMap::all_segments(int N)
{
  int end;
  std::vector<std::vector<int>> res;
  for (int i = 0; i < N; i++)
  {
    for (int j = i + 2; j < N; j++)
    {
      if (i==0) end = N - 1;
      else end = N;
      
      for (int k = j + 2; k < end; k++)
      {
        res.push_back({i,j,k});
      }
    }
  }
  return res;
}

std::vector<std::string> TrojanMap::ThreeOptSwap(std::vector<int> segment, std::vector<std::string>& ids)
{
  int A = segment[0]; int B = segment[0]+1; int C = segment[1];
  int D = segment[1]+1; int E = segment[2]; int F = (segment[2]+1)%(ids.size());
  std::vector<double> dis(8);
  std::vector<std::string> res;
  dis[0] = CalculateDistance(ids[A],ids[B])+CalculateDistance(ids[C],ids[D])+CalculateDistance(ids[E],ids[F]);
  dis[1] = CalculateDistance(ids[E],ids[A])+CalculateDistance(ids[C],ids[D])+CalculateDistance(ids[F],ids[B]);
  dis[2] = CalculateDistance(ids[A],ids[C])+CalculateDistance(ids[D],ids[B])+CalculateDistance(ids[E],ids[F]);
  dis[3] = CalculateDistance(ids[A],ids[B])+CalculateDistance(ids[C],ids[E])+CalculateDistance(ids[D],ids[F]);
  dis[4] = CalculateDistance(ids[D],ids[A])+CalculateDistance(ids[C],ids[E])+CalculateDistance(ids[F],ids[B]);
  dis[5] = CalculateDistance(ids[A],ids[C])+CalculateDistance(ids[B],ids[E])+CalculateDistance(ids[D],ids[F]);
  dis[6] = CalculateDistance(ids[E],ids[A])+CalculateDistance(ids[B],ids[D])+CalculateDistance(ids[F],ids[C]);
  dis[7] = CalculateDistance(ids[D],ids[A])+CalculateDistance(ids[B],ids[E])+CalculateDistance(ids[F],ids[C]);

  auto min = std::min_element(dis.begin(), dis.end());
  int argmin = std::distance(dis.begin(), min);
  if (argmin == 0)
  {
    return res;
  }

  std::vector<std::string> firstsegment;
  if (F==0)
  {
    firstsegment.insert(firstsegment.end(),ids.begin(), ids.begin()+A+1);
  }
  else{
    firstsegment.insert(firstsegment.end(), ids.begin()+F,ids.end());
    firstsegment.insert(firstsegment.end(), ids.begin(),ids.begin()+A+1);
  }
  std::vector<std::string> secondsegment(ids.begin()+B,ids.begin()+C+1);
  std::vector<std::string> thirdsegment(ids.begin()+D,ids.begin()+E+1);
  switch (argmin)
  {
  case 1:
  {  
    std::reverse(firstsegment.begin(), firstsegment.end());
    res.insert(res.end(), firstsegment.begin(), firstsegment.end());
    res.insert(res.end(), secondsegment.begin(), secondsegment.end());
    res.insert(res.end(), thirdsegment.begin(), thirdsegment.end());
    return res;
  }
  case 2:
  {
    res.insert(res.end(), firstsegment.begin(), firstsegment.end());
    std::reverse(secondsegment.begin(), secondsegment.end());
    res.insert(res.end(), secondsegment.begin(), secondsegment.end());
    res.insert(res.end(), thirdsegment.begin(), thirdsegment.end());
    return res;
  }
  case 3:
  {
    res.insert(res.end(), firstsegment.begin(), firstsegment.end());
    res.insert(res.end(), secondsegment.begin(), secondsegment.end());
    std::reverse(thirdsegment.begin(), thirdsegment.end());
    res.insert(res.end(), thirdsegment.begin(), thirdsegment.end());
    return res;
  }
  case 4:
  {
    std::reverse(firstsegment.begin(), firstsegment.end());
    res.insert(res.end(), firstsegment.begin(), firstsegment.end());
    res.insert(res.end(), secondsegment.begin(), secondsegment.end());
    std::reverse(thirdsegment.begin(), thirdsegment.end());
    res.insert(res.end(), thirdsegment.begin(), thirdsegment.end());
    return res;
  }
  case 5:
  {
    res.insert(res.end(), firstsegment.begin(), firstsegment.end());
    std::reverse(secondsegment.begin(), secondsegment.end());
    res.insert(res.end(), secondsegment.begin(), secondsegment.end());
    std::reverse(thirdsegment.begin(), thirdsegment.end());
    res.insert(res.end(), thirdsegment.begin(), thirdsegment.end());
    return res;
  }
  case 6:
  {
    std::reverse(firstsegment.begin(), firstsegment.end());
    res.insert(res.end(), firstsegment.begin(), firstsegment.end());
    std::reverse(secondsegment.begin(), secondsegment.end());
    res.insert(res.end(), secondsegment.begin(), secondsegment.end());
    res.insert(res.end(), thirdsegment.begin(), thirdsegment.end());
    return res;
  }
  case 7:
  {
    std::reverse(firstsegment.begin(), firstsegment.end());
    res.insert(res.end(), firstsegment.begin(), firstsegment.end());
    std::reverse(secondsegment.begin(), secondsegment.end());
    res.insert(res.end(), secondsegment.begin(), secondsegment.end());
    std::reverse(thirdsegment.begin(), thirdsegment.end());
    res.insert(res.end(), thirdsegment.begin(), thirdsegment.end());
    return res;
  }
  }
}

std::vector<std::string> TrojanMap::correct_order(std::vector<std::string> ids, std::string source)
{
  auto it = std::find(ids.begin(), ids.end(), source);
  std::vector<std::string> res(it, ids.end());
  res.insert(res.end(), ids.begin(), it);
  return res;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_3opt(
    std::vector<std::string> location_ids)
{
  std::pair<double, std::vector<std::vector<std::string>>> records;
  if (location_ids.empty())
    return records;
  
  std::string source = location_ids[0];
  location_ids.push_back(source);
  records.first = CalculatePathLength(location_ids);
  records.second.push_back(location_ids);
  int unchanged = 0;

  std::vector<std::vector<int>> all_seg = all_segments(location_ids.size());

  while (unchanged < 60)
  {
    unchanged++;
    for (auto& s : all_seg)
    {
      std::vector<std::string> cur_ids = records.second.back();
      cur_ids.pop_back();
      std::vector<std::string> new_ids = ThreeOptSwap(s, cur_ids);
      if (new_ids.empty())
      {
        continue;
      }
      new_ids = correct_order(new_ids, source);
      new_ids.push_back(source);
      double new_distance = CalculatePathLength(new_ids);
      if (new_distance < records.first)
      {
        unchanged = 0;
        records.first = new_distance;
        records.second.push_back(new_ids);
        break;
      }
    }
  }
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
  std::fstream fin;
  fin.open(locations_filename, std::ios::in);
  std::string line;

  getline(fin, line);
  while (getline(fin, line))
  {
    location_names_from_csv.push_back(line);
  }
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
  std::fstream fin;
  fin.open(dependencies_filename, std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line))
  {
    std::stringstream s(line);
    std::vector<std::string> dep;
    while (getline(s, word, ','))
    {
      dep.push_back(word);
    }
    if (dep.size() == 2)
    {
      dependencies_from_csv.push_back(dep);
    }
  }
  return dependencies_from_csv;
}

/**
 * Check whether a circle existed in directed graph for delivering Trojan function
 */
bool TrojanMap::IsCycleDeliver_helper(std::string loc_name, std::map<std::string, int> &visited)
{
  visited[loc_name] = 1;
  for (auto &child : AdjListDeliver[loc_name])
  {
    if (visited[child] == 1)
    {
      return true;
    }
    else if (visited[child] != 2)
    {
      if (IsCycleDeliver_helper(child, visited))
      {
        return true;
      }
    }
  }
  visited[loc_name] = 2;
  return false;
}

bool TrojanMap::IsCycleDeliver()
{
  std::map<std::string, int> visited;

  for (auto &node : AdjListDeliver)
  {
    if (visited[node.first] != 1 && visited[node.first] != 2)
    {
      if (IsCycleDeliver_helper(node.first, visited))
      {
        return true;
      }
    }
  }
  return false;
}

/**
 * Delivering Trojan Helper
 */
void TrojanMap::DeliverHelper(std::string loc_name,
                              std::map<std::string, int> &marks, std::vector<std::string> &topo)
{
  marks[loc_name] = 1;
  for (auto &child : AdjListDeliver[loc_name])
  {
    if (marks[child] != 1)
    {
      DeliverHelper(child, marks, topo);
    }
  }
  topo.push_back(loc_name);
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
  std::map<std::string, int> marks;

  for (auto &loc : locations)
  {
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
  const Node &node = data[id];
  if (node.lon >= square[0] && node.lon <= square[1] && node.lat <= square[2] && node.lat >= square[3])
  {
    return true;
  }
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
  for (const auto &node : data)
  {
    if (inSquare(node.first, square))
    {
      subgraph.push_back(node.first);
    }
  }
  return subgraph;
}
/**
 * Cycle Detection Helper
 */
bool TrojanMap::CycleHelper(std::string current_id, std::map<std::string, int> &marks,
                            std::string parent_id, std::vector<double> &square,
                            std::map<std::string, std::string> &pre,
                            std::string &end, std::string &start)
{
  marks[current_id] = 1;
  for (const auto &child_id : data[current_id].neighbors)
  {
    if (inSquare(child_id, square))
    {
      if (marks[child_id] == 1 && child_id != parent_id)
      {
        end = current_id;
        start = child_id;
        return true;
      }
      if (marks[child_id] != 1)
      {
        pre[child_id] = current_id;
        if (CycleHelper(child_id, marks, current_id, square, pre, end, start))
        {
          return true;
        }
      }
    }
  }
  return false;
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
  std::map<std::string, int> marks;
  std::map<std::string, std::string> pre;
  std::vector<std::string> path;
  std::string end;
  std::string start;
  for (const auto &node : subgraph)
  {
    if (marks[node] != 1)
    {
      if (CycleHelper(node, marks, "", square, pre, end, start))
      {
        path.push_back(end);
        while (end != start)
        {
          end = pre[end];
          path.push_back(end);
        }
        std::reverse(path.begin(), path.end());
        path.push_back(start);
        CyclePath = path;
        return true;
      }
    }
  }
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
  std::priority_queue<std::pair<double, std::string>,
                      std::vector<std::pair<double, std::string>>>
      q;
  std::string Lid = GetID(name);
  if (Lid.empty() || name.empty() || k <= 0 || r <= 0)
  {
    return res;
  }

  for (auto &node : data)
  {
    if (node.second.name == name)
      continue;
    auto &node_attr = node.second.attributes;
    if (node_attr.find(attributesName) != node_attr.end())
    {
      double dis = CalculateDistance(node.first, Lid);
      if (dis <= r)
      {
        if (q.size() < k)
        {
          q.push(std::make_pair(dis, node.first));
        }
        else
        {
          if (dis < q.top().first)
          {
            q.pop();
            q.push(std::make_pair(dis, node.first));
          }
        }
      }
    }
  }
  while (!q.empty())
  {
    res.push_back(q.top().second);
    q.pop();
  }
  std::reverse(res.begin(), res.end());
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
