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

std::vector<float> UserInputToCoordinates(std::string& input) {
    std::vector<float> coordinates;
    std::istringstream InputStream(input);
    std::string temp;


    while(std::getline(InputStream, temp, ',')) {
        //std::cout << temp << "\n";
        coordinates.push_back(stof(temp));
    }
    for (float i : coordinates) {
        std::cout << i << "\n";
    }
    return coordinates;
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
    
    float start_x;
    float start_y;
    float end_x;
    float end_y;
    std::vector<float> starting_coors;
    std::vector<float> ending_coors;
    std::string input_start;
    std::string input_end;

    // std::cout << "Enter the starting location coordinates (ex. 10, 10): " << "\n";
    // std::getline(std::cin, input_start);

    // starting_coors = UserInputToCoordinates(input_start);
    starting_coors = {10, 10};
    start_x = starting_coors[0];
    start_y = starting_coors[1];

    // std::cout << "Enter the destination location coordinates (ex. 90, 90): " << "\n";
    // std::getline(std::cin, input_end);

    // ending_coors = UserInputToCoordinates(input_end);
    ending_coors = {90, 90};
    end_x = ending_coors[0];
    end_y = ending_coors[1];

    //Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

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
