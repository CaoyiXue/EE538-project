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

// Test FindClosestName function
TEST(TrojanMapTest, FindClosestName)
{
  TrojanMap m;
  EXPECT_EQ(m.FindClosestName("Rolphs"), "Ralphs");
  EXPECT_EQ(m.FindClosestName("Targeety"), "Target");
}

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
  std::vector<std::string> expected{ "Ralphs", "KFC", "Chick-fil-A", "Kaitlyn", "Adams Normandie Historic District" };
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
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs", "KFC"}, {"KFC", "Ralphs"}, {"Ralphs","Chick-fil-A"}};
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