#include <filesystem>
#include <unistd.h>
#include "gtest/gtest.h"
#include "gtest/internal/gtest-filepath.h"
#include "src/lib/trojanmap.h"

TEST(GetLat, GetLatTest1)
{
  TrojanMap tm;
  std::string id = "653740";
  double expected = tm.GetLat(id);
  double actual = 34.0356378;
  EXPECT_EQ(expected, actual);
  std::string wrong_lat = "-1";
  EXPECT_EQ(tm.GetLat(wrong_lat), -1);
}

TEST(GetLon, GetLonTest1)
{
  TrojanMap tm;
  std::string id = "653740";
  double expected = tm.GetLon(id);
  double actual = -118.2690270;
  EXPECT_EQ(expected, actual);
  std::string wrong_lon = "-1";
  EXPECT_EQ(tm.GetLon(wrong_lon), -1);
}
// 358784231,34.0189016,-118.3070196,Foshay Learning Center,{'school'},{'7432340645'}
TEST(GetName, GetNameTest1)
{
  TrojanMap tm;
  std::string id = "358784231";
  std::string expected = tm.GetName(id);
  std::string actual = "Foshay Learning Center";
  EXPECT_EQ(expected, actual);
  std::string wrong_id = "-1";
  EXPECT_EQ(tm.GetName(wrong_id), "NULL");
}
TEST(GetNeighborIDs, GetNeighborIDsTest1)
{
  TrojanMap tm;
  std::string id = "358784231";
  std::vector<std::string> expected = tm.GetNeighborIDs(id);
  std::vector<std::string> actual = {"7432340645"};
  EXPECT_EQ(expected, actual);
  std::string wrong_id = "-1";
  std::vector<std::string> actual2 = {};
  EXPECT_EQ(tm.GetNeighborIDs(wrong_id), actual2);
}

TEST(GetID, GetIDTest1)
{
  TrojanMap tm;
  std::string name = "Foshay Learning Center";
  std::string expected = tm.GetID(name);
  std::string actual = "358784231";
  EXPECT_EQ(expected, actual);
  std::string wrong_name = "-1";
  EXPECT_EQ(tm.GetID(wrong_name), "");
}

TEST(TrojanMapTest, Autocomplete)
{
  TrojanMap m;
  // Test the simple case
  auto names = m.Autocomplete("Chi");
  std::unordered_set<std::string> gt = {"Chick-fil-A", "Chipotle", "Chinese Street Food"}; // groundtruth for "Ch"
  int success = 0;
  for (auto &n : names)
  {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0)
    {
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower case
  names = m.Autocomplete("chi");
  success = 0;
  for (auto &n : names)
  {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0)
    {
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the lower and upper case
  names = m.Autocomplete("cHi");
  success = 0;
  for (auto &n : names)
  {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0)
    {
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
  // Test the upper case
  names = m.Autocomplete("CHI");
  success = 0;
  for (auto &n : names)
  {
    EXPECT_EQ(gt.count(n) > 0, true) << n + " is not in gt.";
    if (gt.count(n) > 0)
    {
      success++;
    }
  }
  EXPECT_EQ(success, gt.size());
}

// Test FindPosition function
TEST(TrojanMapTest, FindPosition)
{
  TrojanMap m;

  // Test Chick-fil-A
  auto position = m.GetPosition("Chick-fil-A");
  std::pair<double, double> gt1(34.0167334, -118.2825307); // groundtruth for "Chick-fil-A"
  EXPECT_EQ(position, gt1);
  // Test Ralphs
  position = m.GetPosition("Ralphs");
  std::pair<double, double> gt2(34.0317653, -118.2908339); // groundtruth for "Ralphs"
  EXPECT_EQ(position, gt2);
  // Test Target
  position = m.GetPosition("Target");
  std::pair<double, double> gt3(34.0257016, -118.2843512); // groundtruth for "Target"
  EXPECT_EQ(position, gt3);
  // Test Unknown
  position = m.GetPosition("XXX");
  std::pair<double, double> gt4(-1, -1);
  EXPECT_EQ(position, gt4);
}

// Test CalculateEditDistance function
TEST(TrojanMapTest, CalculateEditDistance)
{
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance("horse", "ros"), 3);
  EXPECT_EQ(m.CalculateEditDistance("intention", "execution"), 5);
}
TEST(TrojanMapTest, CalculateEditDistance_noDP)
{
  TrojanMap m;
  EXPECT_EQ(m.CalculateEditDistance_noDP("horse", "ros"), 3);
  EXPECT_EQ(m.CalculateEditDistance_noDP("intention", "execution"), 5);
}

// Test FindClosestName function
TEST(TrojanMapTest, FindClosestName)
{
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName("Rolphs"), "Ralphs");
  EXPECT_EQ(m.FindClosestName("Targeety"), "Target");
}

// Phase 2
TEST(ShortestPath, CalculateShortestPathTest1)
{
  TrojanMap m;
  auto path = m.CalculateShortestPath_Dijkstra("Flower Street & Adams Boulevard", "Flower Street & Adams Boulevard 1");
  std::vector<std::string> gt{"8501336171", "8501336172"};
  // Print the path lengths
  std::cout << "My path length: " << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);

  path = m.CalculateShortestPath_Dijkstra("Flower Street & Adams Boulevard 1", "Flower Street & Adams Boulevard");
  std::reverse(gt.begin(), gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: " << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
TEST(ShortestPath, CalculateShortestPathTest2)
{
  TrojanMap m;
  auto path = m.CalculateShortestPath_Bellman_Ford("Flower Street & Adams Boulevard", "Flower Street & Adams Boulevard 1");
  std::vector<std::string> gt{"8501336171", "8501336172"};
  // Print the path lengths
  std::cout << "My path length: " << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);

  path = m.CalculateShortestPath_Bellman_Ford("Flower Street & Adams Boulevard 1", "Flower Street & Adams Boulevard");
  std::reverse(gt.begin(), gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: " << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
// Test CalculateShortestPath_Dijkstra function
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra)
{
  TrojanMap m;

  // Test from Ralphs to Chick-fil-A
  auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Chick-fil-A");
  std::vector<std::string> gt{
      "2578244375", "4380040154", "4380040153", "4380040152", "4380040148", "6818427920", "6818427919",
      "6818427918", "6818427892", "6818427898", "6818427917", "6818427916", "7232024780", "6813416145",
      "6813416154", "6813416153", "6813416152", "6813416151", "6813416155", "6808069740", "6816193785",
      "6816193786", "123152294", "4015203136", "4015203134", "4015203133", "21098539", "6389467809",
      "4015203132", "3195897587", "4015203129", "4015203127", "6352865690", "6813379589", "6813379483",
      "3402887081", "6814958394", "3402887080", "602606656", "4872897515", "4399697589", "6814958391",
      "123209598", "6787673296", "122728406", "6807762271", "4399697304", "4399697302", "5231967015",
      "1862347583", "3233702827", "4540763379", "6819179753", "6820935900", "6820935901", "6813379556",
      "6820935898", "1781230450", "1781230449", "4015405542", "4015405543", "1837212104", "1837212107",
      "2753199985", "6820935907", "1837212100", "4015372458", "6813411588", "1837212101", "6814916516",
      "6814916515", "6820935910", "4547476733"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: " << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);

  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Dijkstra("Chick-fil-A", "Ralphs");
  std::reverse(gt.begin(), gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: " << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}
TEST(GetPredecessors, GetPredecessorsTest1)
{
  TrojanMap m;
  auto pres = m.GetPredecessors();
  std::vector<std::string> expected1 = {"123295515"};
  EXPECT_EQ(pres["6252360750"], expected1);
  std::vector<std::string> expected2 = {"6817369216", "123076583", "123530460"};
  EXPECT_EQ(pres["123530463"], expected2);
  EXPECT_EQ(m.data.size(), pres.size());
}

// Test CalculateShortestPath_Bellman_Ford function
TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford)
{
  TrojanMap m;

  // Test from Ralphs to Chick-fil-A
  auto path = m.CalculateShortestPath_Bellman_Ford("Ralphs", "Chick-fil-A");
  std::vector<std::string> gt{
      "2578244375", "4380040154", "4380040153", "4380040152", "4380040148", "6818427920", "6818427919",
      "6818427918", "6818427892", "6818427898", "6818427917", "6818427916", "7232024780", "6813416145",
      "6813416154", "6813416153", "6813416152", "6813416151", "6813416155", "6808069740", "6816193785",
      "6816193786", "123152294", "4015203136", "4015203134", "4015203133", "21098539", "6389467809",
      "4015203132", "3195897587", "4015203129", "4015203127", "6352865690", "6813379589", "6813379483",
      "3402887081", "6814958394", "3402887080", "602606656", "4872897515", "4399697589", "6814958391",
      "123209598", "6787673296", "122728406", "6807762271", "4399697304", "4399697302", "5231967015",
      "1862347583", "3233702827", "4540763379", "6819179753", "6820935900", "6820935901", "6813379556",
      "6820935898", "1781230450", "1781230449", "4015405542", "4015405543", "1837212104", "1837212107",
      "2753199985", "6820935907", "1837212100", "4015372458", "6813411588", "1837212101", "6814916516",
      "6814916515", "6820935910", "4547476733"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: " << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);

  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Bellman_Ford("Chick-fil-A", "Ralphs");
  std::reverse(gt.begin(), gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: " << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

/*
TEST(ReadLocationsFromCSVFile, ReadLocationsFromCSVFileTest1)
{
  TrojanMap m;
  // need manually modify file path
  std::string file_path = "/Users/caoyi/EE538/EE538-project/input/topologicalsort_locations.csv";
  std::vector<std::string> expected{"Ralphs", "KFC", "Chick-fil-A"};
  EXPECT_EQ(m.ReadLocationsFromCSVFile(file_path), expected);
}
TEST(ReadLocationsFromCSVFile, ReadLocationsFromCSVFileTest2)
{
  TrojanMap m;
  // need manually modify file path
  std::string file_path = "/Users/caoyi/EE538/EE538-project/input/test1_l.csv";
  std::vector<std::string> expected{"Ralphs", "KFC", "Chick-fil-A", "Kaitlyn", "Adams Normandie Historic District"};
  EXPECT_EQ(m.ReadLocationsFromCSVFile(file_path), expected);
}

TEST(ReadDependenciesFromCSVFile, ReadDependenciesFromCSVFileTest1)
{
  TrojanMap m;
  // testing::internal::FilePath path = testing::internal::FilePath::GetCurrentDir();
  // need manually modify file path
  std::string file_path = "/Users/caoyi/EE538/EE538-project/input/topologicalsort_dependencies.csv";
  std::vector<std::vector<std::string>> expected{{"Ralphs", "Chick-fil-A"}, {"Ralphs", "KFC"}, {"Chick-fil-A", "KFC"}};
  EXPECT_EQ(m.ReadDependenciesFromCSVFile(file_path), expected);
}
TEST(ReadDependenciesFromCSVFile, ReadDependenciesFromCSVFileTest2)
{
  TrojanMap m;
  // testing::internal::FilePath path = testing::internal::FilePath::GetCurrentDir();
  // need manually modify file path
  std::string file_path = "/Users/caoyi/EE538/EE538-project/input/test1_d.csv";
  std::vector<std::vector<std::string>> expected{{"Ralphs", "KFC"}, {"Ralphs", "Chick-fil-A"}, {"Kaitlyn", "Chick-fil-A"}, {"Kaitlyn", "Adams Normandie Historic District"}};
  EXPECT_EQ(m.ReadDependenciesFromCSVFile(file_path), expected);
}
*/

// Test cycle detection function
TEST(TrojanMapTest, TopologicalSort)
{
  TrojanMap m;

  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs", "KFC"}, {"Ralphs", "Chick-fil-A"}, {"KFC", "Chick-fil-A"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt = {"Ralphs", "KFC", "Chick-fil-A"};
  EXPECT_EQ(result, gt);
}
TEST(TrojanMapTest, TopologicalSortTest2)
{
  TrojanMap m;

  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs", "KFC"}, {"Ralphs", "Chick-fil-A"}, {"Chick-fil-A", "KFC"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt = {"Ralphs", "Chick-fil-A", "KFC"};
  EXPECT_EQ(result, gt);
}
TEST(TrojanMapTest, TopologicalSortTest3)
{
  TrojanMap m;

  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC", "Kaitlyn", "FaceHaus"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs", "KFC"}, {"Ralphs", "Chick-fil-A"}, {"Kaitlyn", "Chick-fil-A"}, {"Kaitlyn", "FaceHaus"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt = {"Kaitlyn", "FaceHaus", "Ralphs", "Chick-fil-A", "KFC"};
  EXPECT_EQ(result, gt);
}
TEST(TrojanMapTest, TopologicalSortTest4)
{
  TrojanMap m;

  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs", "KFC"}, {"KFC", "Chick-fil-A"}, {"Chick-fil-A", "Ralphs"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt;
  EXPECT_EQ(result, gt);
}
TEST(TrojanMapTest, TopologicalSortTest5)
{
  TrojanMap m;

  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs", "KFC"}, {"KFC", "Ralphs"}, {"Ralphs", "Chick-fil-A"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt;
  EXPECT_EQ(result, gt);
}

// Test cycle detection function
TEST(TrojanMapTest, CycleDetection)
{
  TrojanMap m;

  // Test case 1
  std::vector<double> square1 = {-118.299, -118.264, 34.032, 34.011};
  auto sub1 = m.GetSubgraph(square1);
  bool result1 = m.CycleDetection(sub1, square1);
  EXPECT_EQ(result1, true);

  // Test case 2
  std::vector<double> square2 = {-118.290, -118.289, 34.030, 34.020};
  auto sub2 = m.GetSubgraph(square2);
  bool result2 = m.CycleDetection(sub2, square2);
  EXPECT_EQ(result2, false);
}

// Phase 3
// Test TSP function
TEST(TrojanMapTest, TSP1)
{
  TrojanMap m;

  std::vector<std::string> input{"6819019976", "6820935923", "122702233", "8566227783", "8566227656", "6816180153", "1873055993", "7771782316"}; // Input location ids
  auto result = m.TravellingTrojan_Brute_force(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl;                                                                                  // Print the result path lengths
  std::vector<std::string> gt{"6819019976", "1873055993", "8566227656", "122702233", "8566227783", "6816180153", "7771782316", "6820935923", "6819019976"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;                                                                     // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size() - 1]) // clockwise
    flag = true;
  std::reverse(gt.begin(), gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size() - 1])
    flag = true;

  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, TSP2)
{
  TrojanMap m;

  std::vector<std::string> input{"6819019976", "6820935923", "122702233", "8566227783", "8566227656", "6816180153", "1873055993", "7771782316"}; // Input location ids
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl;                                                                                  // Print the result path lengths
  std::vector<std::string> gt{"6819019976", "1873055993", "8566227656", "122702233", "8566227783", "6816180153", "7771782316", "6820935923", "6819019976"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;                                                                     // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size() - 1]) // clockwise
    flag = true;
  std::reverse(gt.begin(), gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size() - 1])
    flag = true;

  EXPECT_EQ(flag, true);
}

TEST(TwoOptSwap, TwoOptSwapTest1)
{
  TrojanMap m;

  std::vector<std::string> input{"0", "1", "2", "3", "4", "5", "6", "7", "0"};
  input = m.TwoOptSwap(4, 7, input);
  std::vector<std::string> expected{"0", "1", "2", "3", "7", "6", "5", "4", "0"};
  EXPECT_EQ(input, expected);
}

TEST(TrojanMapTest, TSP3)
{
  TrojanMap m;

  std::vector<std::string> input{"6819019976", "6820935923", "122702233", "8566227783", "8566227656", "6816180153", "1873055993", "7771782316"}; // Input location ids
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl;                                                                                  // Print the result path lengths
  std::vector<std::string> gt{"6819019976", "1873055993", "8566227656", "122702233", "8566227783", "6816180153", "7771782316", "6820935923", "6819019976"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;                                                                     // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size() - 1]) // clockwise
    flag = true;
  std::reverse(gt.begin(), gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size() - 1])
    flag = true;

  EXPECT_EQ(flag, true);
}

TEST(all_segments, all_segmentsTest1)
{
  TrojanMap m;
  std::vector<std::vector<int>> expected = {{0, 2, 4}, {0, 2, 5}, {0, 2, 6}, {0, 2, 7}, {0, 3, 5}, {0, 3, 6}, {0, 3, 7}, {0, 4, 6}, {0, 4, 7}, {0, 5, 7}, {1, 3, 5}, {1, 3, 6}, {1, 3, 7}, {1, 3, 8}, {1, 4, 6}, {1, 4, 7}, {1, 4, 8}, {1, 5, 7}, {1, 5, 8}, {1, 6, 8}, {2, 4, 6}, {2, 4, 7}, {2, 4, 8}, {2, 5, 7}, {2, 5, 8}, {2, 6, 8}, {3, 5, 7}, {3, 5, 8}, {3, 6, 8}, {4, 6, 8}};
  auto result = m.all_segments(9);
  EXPECT_EQ(result, expected);
}

TEST(correct_order, correct_orderTest1)
{
  TrojanMap m;
  std::vector<std::string> input = {"8566227783", "8566227656", "6819019976", "6820935923", "122702233"};
  std::vector<std::string> expected = {"6819019976", "6820935923", "122702233", "8566227783", "8566227656"};
  std::string source = "6819019976";
  auto result = m.correct_order(input, source);
  EXPECT_EQ(result, expected);
}

TEST(TrojanMapTest, TSP3optTest1)
{
  TrojanMap m;

  std::vector<std::string> input{"6819019976", "6820935923", "122702233", "8566227783", "8566227656", "6816180153", "1873055993", "7771782316"}; // Input location ids
  auto result = m.TravellingTrojan_3opt(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl;                                                                                  // Print the result path lengths
  std::vector<std::string> gt{"6819019976", "1873055993", "8566227656", "122702233", "8566227783", "6816180153", "7771782316", "6820935923", "6819019976"}; // Expected order
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;                                                                     // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size() - 1]) // clockwise
    flag = true;
  std::reverse(gt.begin(), gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size() - 1])
    flag = true;

  EXPECT_EQ(flag, true);
}

// Test FindNearby points
TEST(TrojanMapTest, FindNearby)
{
  TrojanMap m;

  auto result = m.FindNearby("supermarket", "Ralphs", 10, 10);
  std::vector<std::string> ans{"5237417649", "6045067406", "7158034317"};
  EXPECT_EQ(result, ans);
}