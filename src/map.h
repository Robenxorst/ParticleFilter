/*
 * map.h
 * класс карты;
 */

#ifndef MAP_H_
#define MAP_H_
// объявляем класс карты, в которой содержится структура единичного объекта
class Map {
public:
	
	struct single_landmark_s{

		int id_i; // Landmark ID - идентификатор объекта(конус, дерево,столб, автобусная остановка);
		float x_f; // Landmark x-position in the map (global coordinates)
		float y_f; // Landmark y-position in the map (global coordinates)
	};
//    создаем без заполнения динамический массив шаблонного класса вектор, состоящий из структур
//    single_landmark_s;
	std::vector<single_landmark_s> landmark_list ; // List of landmarks in the map

};

#endif /* MAP_H_ */
