#include <iostream>
#include "src/lib/trojanmap.h"
#include <time.h>

int main()
{
    TrojanMap map;
    // {
    //     std::string menu = "Input :";
    //     std::cout << menu;
    //     std::string input;
    //     getline(std::cin, input);
    //     menu = "Out for FindClosestName_noDP:";
    //     std::cout << menu << std::endl;
    //     auto start = std::chrono::high_resolution_clock::now();
    //     auto results = map.FindClosestName_noDP(input);
    //     auto stop = std::chrono::high_resolution_clock::now();
    //     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    //     std::cout << "Did you mean " << results << " instead of " << input << std::endl;
    //     std::cout << "Time taken by function: " << duration.count() / 1000 << " ms" << std::endl
    //               << std::endl;

    //     menu = "Out for FindClosestName:";
    //     std::cout << menu << std::endl;
    //     start = std::chrono::high_resolution_clock::now();
    //     results = map.FindClosestName(input);
    //     stop = std::chrono::high_resolution_clock::now();
    //     duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    //     std::cout << "Did you mean " << results << " instead of " << input << std::endl;
    //     std::cout << "Time taken by function: " << duration.count() / 1000 << " ms" << std::endl
    //               << std::endl;
    // }

    {
        std::string menu = "In this task, we will select N random points on the map and you need to find the path to travel these points and back to the start point.";
        std::cout << menu << std::endl
                  << std::endl;
        menu = "Please input the number of the places:";
        std::cout << menu;
        std::string input;
        getline(std::cin, input);
        int num = std::stoi(input);
        std::vector<std::string> keys;
        for (auto x : map.data)
        {
            keys.push_back(x.first);
        }
        std::vector<std::string> locations;
        srand(time(NULL));
        for (int i = 0; i < num; i++)
            locations.push_back(keys[rand() % keys.size()]);
        for (auto x : locations)
            std::cout << "\"" << x << "\",";
        std::cout << "\nCalculating ..." << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        auto results = map.TravellingTrojan_Backtracking(locations);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

        menu = "*************************Results******************************\n";
        std::cout << menu;
        menu = "TravellingTrojan_Brute_force\n";
        std::cout << menu;
        if (results.second.size() != 0)
        {
            for (auto x : results.second[results.second.size() - 1])
                std::cout << "\"" << x << "\",";
            std::cout << "\nThe distance of the path is:" << results.first << " miles" << std::endl;
        }
        else
        {
            std::cout << "The size of the path is 0" << std::endl;
        }
        std::cout << "Time taken by function: " << duration.count() / 1000 << " ms" << std::endl
                  << std::endl;

        std::cout << "Calculating ..." << std::endl;
        start = std::chrono::high_resolution_clock::now();
        results = map.TravellingTrojan_Backtracking(locations);
        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        menu = "*************************Results******************************\n";
        std::cout << menu;
        menu = "TravellingTrojan_Backtracking\n";
        std::cout << menu;
        if (results.second.size() != 0)
        {
            for (auto x : results.second[results.second.size() - 1])
                std::cout << "\"" << x << "\",";
            std::cout << "\nThe distance of the path is:" << results.first << " miles" << std::endl;
        }
        else
        {
            std::cout << "The size of the path is 0" << std::endl;
        }
        std::cout << "Time taken by function: " << duration.count() / 1000 << " ms" << std::endl
                  << std::endl;

        std::cout << "Calculating ..." << std::endl;
        start = std::chrono::high_resolution_clock::now();
        results = map.TravellingTrojan_2opt(locations);
        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        menu = "*************************Results******************************\n";
        std::cout << menu;
        menu = "TravellingTrojan_2opt\n";
        std::cout << menu;
        if (results.second.size() != 0)
        {
            for (auto x : results.second[results.second.size() - 1])
                std::cout << "\"" << x << "\",";
            std::cout << "\nThe distance of the path is:" << results.first << " miles" << std::endl;
        }
        else
        {
            std::cout << "The size of the path is 0" << std::endl;
        }
        std::cout << "Time taken by function: " << duration.count() / 1000 << " ms" << std::endl
                  << std::endl;

        std::cout << "Calculating ..." << std::endl;
        start = std::chrono::high_resolution_clock::now();
        results = map.TravellingTrojan_3opt(locations);
        stop = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        menu = "*************************Results******************************\n";
        std::cout << menu;
        menu = "TravellingTrojan_3opt\n";
        std::cout << menu;
        if (results.second.size() != 0)
        {
            for (auto x : results.second[results.second.size() - 1])
                std::cout << "\"" << x << "\",";
            std::cout << "\nThe distance of the path is:" << results.first << " miles" << std::endl;
        }
        else
        {
            std::cout << "The size of the path is 0" << std::endl;
        }
        std::cout << "Time taken by function: " << duration.count() / 1000 << " ms" << std::endl
                  << std::endl;
    }
}
