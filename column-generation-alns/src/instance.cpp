#include "instance.hpp"

std::istream &operator>>(std::istream &in, VehicleType &obj)
{
    in >> 
    obj.d_idVehicleType >>
    obj.d_costFixed >>
    obj.d_costVariable >>
    obj.d_speed >>
    obj.d_capacity >>
    obj.d_alpha >>
    obj.d_beta >>
    obj.d_timeMax >>
    obj.d_timeDriveMax;
    return in;
}

std::ostream &operator <<(std::ostream &out, VehicleType &obj)
{
    out <<
    obj.d_idVehicleType << ", " <<
    obj.d_costFixed << ", " <<
    obj.d_costVariable << ", " <<
    obj.d_speed << ", " <<
    obj.d_capacity << ", " <<
    obj.d_alpha << ", " <<
    obj.d_beta << ", " <<
    obj.d_timeMax << ", " <<
    obj.d_timeDriveMax;
    return out;
}


std::istream &operator>>(std::istream &in, VehicleTypeInfo &obj)
{
    in >> 
    obj.d_idVehicleType >>
    obj.d_idDepot >>
    obj.d_nVehicles;
    return in;
}

std::ostream &operator <<(std::ostream &out, VehicleTypeInfo &obj)
{
    out <<
    obj.d_idVehicleType << ", " <<
    obj.d_idDepot << ", " <<
    obj.d_nVehicles;
    return out;
}


std::istream &operator>>(std::istream &in, ScheduledLine &obj)
{
    in >>
    obj.d_idFrom >>
    obj.d_idTo >>
    obj.d_costRequest >>
    obj.d_capacity >>
    obj.d_timeDep >>
    obj.d_timeArr;
    return in;
}

std::ostream &operator <<(std::ostream &out, ScheduledLine &obj)
{
    out <<
    obj.d_idFrom << ", " <<
    obj.d_idTo << ", " <<
    obj.d_costRequest << ", " <<
    obj.d_capacity << ", " <<
    obj.d_timeDep << ", " <<
    obj.d_timeArr;
    return out;
}


std::istream &operator>>(std::istream &in, Node &obj)
{
    in >> 
    obj.d_idNode >>
    obj.d_idDepot >>
    obj.d_latitude >>
    obj.d_longitude >>
    obj.d_type >>
    obj.d_timeServ >>
    obj.d_qDel >>
    obj.d_qCol >>
    obj.d_twDepotStart >>
    obj.d_twDepotEnd >>
    obj.d_twCustStart >>
    obj.d_twCustEnd;
    return in;
}

std::ostream &operator <<(std::ostream &out, Node &obj)
{
    out <<
    obj.d_idNode << ", " <<
    obj.d_idDepot << ", " <<
    obj.d_latitude << ", " <<
    obj.d_longitude << ", " <<
    obj.d_type << ", " <<
    obj.d_timeServ << ", " <<
    obj.d_qDel << ", " <<
    obj.d_qCol << ", " <<
    obj.d_twDepotStart << ", " <<
    obj.d_twDepotEnd << ", " <<
    obj.d_twCustStart << ", " <<
    obj.d_twCustEnd;
    return out;
}


bool Instance::load(const std::string &path_to_instance)
{
    // Load instance file from path
    std::string line;
    std::string headerLine;

    // Open file
    std::ifstream read_file;
    read_file.open(path_to_instance);

    if (!read_file.is_open())
    {
        std::cout << "ERROR: Could not open file: "<< path_to_instance << std::endl;
        return false;
    }

    // Read header
    if (!read_file.eof())
    {
        std::getline(read_file, headerLine, ',');
        std::stringstream ss_header(headerLine);
        ss_header >> d_name >> d_nVehTypes >> 
        d_nVehTypesInfo >> d_nSL >> 
        d_nNodes;
    }
    else
    {
        std::cout << "ERROR: File does not contain any data. File: " << path_to_instance << std::endl;
        return false;
    }
    
    // Read vehicle types
    std::getline(read_file, line); // Header line
    for (int i = 0; i < d_nVehTypes; i++)
    {
        if (!read_file.eof())
        {
            std::getline(read_file, line);
            std::istringstream iss(line);
            VehicleType veh_type_of_line = VehicleType();
            iss >> veh_type_of_line;
            veh_type_of_line.d_speed = veh_type_of_line.d_speed / 3600;
            d_vehTypes.push_back(veh_type_of_line);
        }
        else
        {
            std::cout << "ERROR: File ended earlier then expected in vehicle types. File: " << path_to_instance << std::endl;
            std::cout << "ERROR: The number on top of the instance file does not correspond to the number of lines in the instance file." << std::endl;
            return false;
        }
    }
    std::getline(read_file, line); // Empty line

    // Read vehicle types info
    std::getline(read_file, line); // Header line

    for (int i = 0; i < d_nVehTypesInfo; i++)
    {
        if (!read_file.eof())
        {
            std::getline(read_file, line);
            std::istringstream iss(line);
            VehicleTypeInfo veh_type_info_of_line = VehicleTypeInfo();
            iss >> veh_type_info_of_line;
            // Relax if given
            d_vehTypesInfo.push_back(veh_type_info_of_line);
        }
        else
        {
            std::cout << "ERROR: File ended earlier then expected in vehicle types INFO. File: " << path_to_instance << std::endl;
            std::cout << "ERROR: The number on top of the instance file does not correspond to the number of lines in the instance file." << std::endl;
            
            return false;
        }
    }

    std::getline(read_file, line); // Empty line

    // Read Scheduled lines
    std::getline(read_file, line); // Header line

    for (unsigned short i = 0; i < d_nSL; i++)
    {
        if (!read_file.eof())
        {
            std::getline(read_file, line);
            std::istringstream iss(line);
            ScheduledLine scheduledLine = ScheduledLine();
            iss >> scheduledLine;
            d_slMap[std::make_tuple(scheduledLine.d_idFrom, scheduledLine.d_idTo)].push_back(i);
            d_scheduledLines.push_back(scheduledLine);
        }
        else
        {
            std::cout << "ERROR: File ended earlier then expected in scheduled lines. File: " << path_to_instance << std::endl;
            std::cout << "ERROR: The number on top of the instance file does not correspond to the number of lines in the instance file." << std::endl;
            
            return false;
        }
    }
    std::getline(read_file, line); // Empty line

    // Read nodes
    std::getline(read_file, line); // Header line

    d_nDepots = 0;
    d_nCustNodes = 0;

    for (int i = 0; i < d_nNodes; i++)
    {
        if (!read_file.eof())
        {
            std::getline(read_file, line);
            std::istringstream iss(line);
            Node node = Node();
            iss >> node;
            d_nodes.push_back(node);

            if (node.d_type == -1)
            {
                d_nDepots++;
                d_maxTimeHorizon = node.d_twDepotEnd;
            }
            else
            {
                d_nCustNodes++;
            }
        }
        else
        {
            std::cout << "ERROR: File ended earlier then expected in customer data. File: " << path_to_instance << std::endl;
            std::cout << "ERROR: The number on top of the instance file does not correspond to the number of lines in the instance file." << std::endl;
            
            return false;
        }
    }

    read_file.close();

    return true;
}

std::ostream &operator <<(std::ostream &out, Instance &obj)
{
    out << "Instance: " << obj.d_name << "\n\n";

    for (VehicleType veh_type : obj.d_vehTypes)
    {
        out << veh_type << "\n";
    }
    out << "\n";

    for (VehicleTypeInfo veh_type_info : obj.d_vehTypesInfo)
    {
        out << veh_type_info << "\n";
    }
    out << "\n";

    for (ScheduledLine sl : obj.d_scheduledLines)
    {
        out << sl << "\n";
    }
    out << "\n";

    return out;
}