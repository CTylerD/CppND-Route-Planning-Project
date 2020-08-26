#include <optional>
#include <fstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cfloat>
#include <sstream>
#include "io2d.h"
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;

    auto size = is.tellg();
    std::vector<std::byte> contents(size);

    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

std::vector<float> UserInputToCoordinates(std::string input) {
    std::vector<float> coordinates;
    std::istringstream InputStream(input);
    std::string temp;


    while(std::getline(InputStream, temp, ',')) {
        coordinates.push_back(stof(temp));
    }
    return coordinates;
}

std::vector<float> GetUserCoordinates() {
    float start_x;
    float start_y;
    float end_x;
    float end_y;
    bool valid_start = false;
    bool valid_end = false;
    while (valid_start == false) {
        std::cout << "Please enter your starting location's coordinates (ex. 10, 10): " << "\n";
        std::string user_start_input;
        std::getline(std::cin, user_start_input);
        std::vector<float> input_start = UserInputToCoordinates(user_start_input);
        start_x = input_start[0];
        start_y = input_start[1];

        if (typeid(start_x) == typeid(float) && start_x >= 0 && start_x <= 100 && typeid(start_y) == typeid(float) && start_y >= 0 && start_y <= 100) {
            valid_start = true;
        } else {
            std::cout << "Your starting coordinate values must be integers from 0 to 100." << "\n";
        }
    }

    while (valid_end == false) {
        std::cout << "Please enter your ending location's coordinates (ex. 10, 10): " << "\n";
        std::string user_end_input;
        std::getline(std::cin, user_end_input);
        std::vector<float> input_end = UserInputToCoordinates(user_end_input);
        end_x = input_end[0];
        
        end_y = input_end[1];

        if (typeid(end_x) == typeid(float) && end_x >= 0 && end_x <= 100 && typeid(end_y) == typeid(float) && end_y >= 0 && end_y <= 100) {
            valid_end = true;
        } else {
            std::cout << "Your ending coordinate values must be integers from 0 to 100." << "\n";;
        }
    }
    return {start_x, start_y, end_x, end_y};
}




int main(int argc, const char **argv) {
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }

    std::vector<std::byte> osm_data;

    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }

    std::vector<float> user_coors;

    // uncomment these for faster iteration during development
    // ending_coors = {90, 90};
    // starting_coors = {10, 10};

    user_coors = GetUserCoordinates();

    float start_x = user_coors[0];
    float start_y = user_coors[1];
    float end_x = user_coors[2];
    float end_y = user_coors[3];

    //Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
