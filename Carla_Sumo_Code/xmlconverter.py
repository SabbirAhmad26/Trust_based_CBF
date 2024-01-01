import lxml.etree as ET
import pandas as pd
import argparse

# Function to create XML structure for a vehicle
def create_vehicle_xml(vehicle_id, vehicle_type, departure_time, departure_lane, route_edge1,route_edge2):
    vehicle_elem = ET.Element("vehicle")
    vehicle_elem.set("id", str(vehicle_id))
    vehicle_elem.set("type", vehicle_type)
    vehicle_elem.set("depart", str(departure_time))
    vehicle_elem.set("departLane", departure_lane)

    route_elem = ET.SubElement(vehicle_elem, "route")
    route_elem.set("edges", route_edge1+" "+route_edge2)
    
    return ET.tostring(vehicle_elem, encoding="unicode")

# Sample data
#car_id = 2
#car_type = "vehicle.micro.microlino"
#departure_time = 2.45
#departure_lane = "best"
#route_edges = "0.0.00 -26.0.00"
parser = argparse.ArgumentParser(description='parser for parsing args')
parser.add_argument('--dataset', help='Path to the input file', default='./Dataset/Mitigation_dataset/dataset_init.xlsx')
args = parser.parse_args()
file_path = args.dataset
data = pd.read_excel(file_path)
data_array = data.values

xml_strings =[]

for car in range(len(data_array)):
    dataset = len(data_array)
    vehicle_id = data_array[car][0]
    vehicle_type = data_array[car][1]
    departure_time = data_array[car][2]
    departure_lane = data_array[car][3]
    route_edge1 = data_array[car][4]
    route_edge2 = data_array[car][5]
    vehicle_xml = create_vehicle_xml(vehicle_id, vehicle_type, departure_time, departure_lane, route_edge1,route_edge2)
    #xml_string = ET.tostring(vehicle_xml, encoding="unicode")
    xml_strings.append(vehicle_xml)


namespace = "http://www.w3.org/2001/XMLSchema-instance"
schema_location = "http://sumo.dlr.de/xsd/routes_file.xsd"

root = ET.Element("routes", attrib={
    ET.QName(namespace, "noNamespaceSchemaLocation"): schema_location
})

for xml_string in xml_strings:
    vehicle = ET.XML(xml_string)
    root.append(vehicle)

xml_tree = ET.ElementTree(root)
xml_tree.write("./examples/rou/Town05.rou.xml", pretty_print=True, xml_declaration=True, encoding="utf-8")


    	
