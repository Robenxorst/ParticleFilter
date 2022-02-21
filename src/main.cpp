/*
 * main.cpp
 * Считывает информацию из текстовиков и запускает фильтр Частиц
 */

#include <iostream>
#include <ctime>// подключение блока времени clock
#include <iomanip>//библиотека для вывода чисел через cout << - задаем точность, ширину и т.п.
#include <random>// подключение блока генерации случайных чисел

#include "particle_filter.h"
#include "helper_functions.h"

int main() {

    // Start timer.
    int start = static_cast<int>(clock());// фиксируем время на старте программы

	// parameters related to grading - параметры классификации
    int time_steps_before_lock_required = 100; // number of time steps before accuracy is checked by grader.
	//количество временных шагов прежде чем точность будет проверена сортировщиком
    double max_runtime = 45; // Max allowable runtime to pass [sec]
	//Максимально допустимое время работы фильтра [сек]
    double max_translation_error = 1; // Max allowable translation error to pass [m]
	//Максимальная допустимая ошибка преобразования для передачи [м]
    double max_yaw_error = 0.05; // Max allowable yaw error [rad]
    //максимально допустимая ошибка угла рысканья
	
	//Set up parameters here
    // время прошедшее между измерениями
	double delta_t = 0.1; // Time elapsed between measurements [sec]
    //радиус обнаружения ориентиров
    double sensor_range = 50; // Sensor range [m]
	
	/*
	 * Sigmas - just an estimate, usually comes from uncertainty of sensor, but
	 * if you used fused data from multiple sensors, it's difficult to find
	 * these uncertainties directly.
	 * Сигмы представляют собой погрешности измерений. Для позиции робота
	 * это погрешности по x,y,row, а для ориентира только по x и y
	 */
    // Погрешности GPS
	double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
	// Погрешности измерений ориентира
    double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

    // создаем помехи для работы наших датчиков

    // noise generation - генератор шума
	std::default_random_engine gen;
    // шаблонный класс нормального гауссового распределения,
    std::normal_distribution<double> N_x_init(0, sigma_pos[0]);
	std::normal_distribution<double> N_y_init(0, sigma_pos[1]);
	std::normal_distribution<double> N_theta_init(0, sigma_pos[2]);
	std::normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
	std::normal_distribution<double> N_obs_y(0, sigma_landmark[1]);
	double n_x, n_y, n_theta;
	// Read map data
	Map map; // создаем класс карты
	if (!read_map_data("data/map_data.txt", map)) {
        // читаем, если это возможно
        // текстовик с картой и заполняем класс map
		std::cout << "Error: Could not open map file" << std::endl;
		return -1;
	}

	// Read position data
	std::vector<control_s> position_meas; // создаем массив с контрольными данными
    // Для каждого временного шага имеем свой набор данных о скоростях робота
	if (!read_control_data("data/control_data.txt", position_meas)) {
		std::cout << "Error: Could not open position/control measurement file" << std::endl;
		return -1;
	}
	
	// Read ground truth data
	std::vector<ground_truth> gt;//массив с правдивым положением автомобиля
	if (!read_gt_data("data/gt_data.txt", gt)) {
		std::cout << "Error: Could not open ground truth data file" << std::endl;
		return -1;
	}
	
	// Run particle filter!
    // Запуск фильтра частиц
	int num_time_steps = static_cast<int>(position_meas.size());
	ParticleFilter pf;
	double total_error[3] = {0,0,0};
	double cum_mean_error[3] = {0,0,0};//ошибки рассогласования
	
	for (int i = 0; i < num_time_steps; ++i) {
        std::cout << "Time step: " << i << std::endl;
        // Read in landmark observations for current time step.
        std::ostringstream file;// создаем выходной строковый поток в оперативной памяти(o - out, i - in, io - и то, и то);
        file << "data/observation/observations_" << std::setfill('0') << std::setw(6) << i + 1 << ".txt";
        std::vector<LandmarkObs> observations;// создаем массив с наблюдениями(observations)
        if (!read_landmark_data(file.str(), observations)) {
            std::cout << "Error: Could not open observation file " << i + 1 << std::endl;
            return -1;
        }

        // Initialize particle filter if this is the first time step.
        if (!pf.initialized()) {// если фильтр частиц еще не инициализирован(при первой итерации)
            n_x = N_x_init(gen); // создаем шум через Гауссово распределение
            n_y = N_y_init(gen);
            n_theta = N_theta_init(gen);
            // инициализируем фильтр зашумленными данными
            pf.init(gt[i].x + n_x, gt[i].y + n_y, gt[i].theta + n_theta, sigma_pos);
        } else {// Иначе предсказываем следующее положение робота (без шумов)
            // Predict the vehicle's next state (noiseless).
            pf.prediction(delta_t, sigma_pos, position_meas[i - 1].velocity, position_meas[i - 1].yawrate);
        }
        // добавляем шумы к наблюдаемым данным(observation)
        // simulate the addition of noise to noiseless observation data.
        std::vector<LandmarkObs> noisy_observations;// наблюдения с шумами
        LandmarkObs obs; // экземпляр объекта на карте
        for (size_t j = 0; j < observations.size(); ++j) {
            n_x = N_obs_x(gen);
            n_y = N_obs_y(gen);
            obs = observations[j];
            obs.x = obs.x + n_x;
            obs.y = obs.y + n_y;
            noisy_observations.push_back(obs);
        }

        // Update the weights and resample
        // Обновляем веса и пересобираем фильтр частиц
        pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
        pf.resample();

        // Calculate and output the average weighted error
        // of the particle filter over all time steps so far.
        // Рассчитываем и выводим средневзвешенную ошибку
        // фильтра твердых частиц за все временные интервалы.
        std::vector<Particle> particles = pf.particles;//копируем массив частиц из pf
        int num_particles = static_cast<int>(particles.size());
        // Находим частицу с наибольшим весом(высокая вероятность истинности)
        // Finding the particle with the highest weight(high probability of being true)
        double highest_weight = 0.0;
        Particle best_particle;
        for (int l = 0; l < num_particles; ++l) {
            if (particles[l].weight > highest_weight) {
                highest_weight = particles[l].weight;
                best_particle = particles[l];
            }
        }
        // создаем массив рассогласований между истинным положением
        // и best_particle по x, y, theta;
        // create an array of errors by x, y, theta;
        double *avg_error = getError(gt[i].x, gt[i].y, gt[i].theta, best_particle.x, best_particle.y,
                                     best_particle.theta);

        // Вычисляем кумулятивную взвешенную ошибку
        for (int j = 0; j < 3; ++j) {
            total_error[j] += avg_error[j];
            cum_mean_error[j] = total_error[j] / (double) (i + 1);
        }

        // Print the cumulative weighted error
        // Распечатываем кумулятивную взвешенную ошибку
        std::cout << "Cumulative mean weighted error: x " << cum_mean_error[0] << " y " << cum_mean_error[1] << " yaw "
             << cum_mean_error[2] << std::endl;

        // If the error is too high, say so and then exit.
        // Если ошибка слишком высока, говорим об этом и выходим.
        if (i >= time_steps_before_lock_required) { // проверяем, что сделали больше временных шагов(i), чем указано в ограничителе
            if (cum_mean_error[0] > max_translation_error || cum_mean_error[1] > max_translation_error ||
                cum_mean_error[2] > max_yaw_error) {
                if (cum_mean_error[0] > max_translation_error) {
                    std::cout << "Your x error, " << cum_mean_error[0] << " is larger than the maximum allowable error, "
                         << max_translation_error << std::endl;
                } else if (cum_mean_error[1] > max_translation_error) {
                    std::cout << "Your y error, " << cum_mean_error[1] << " is larger than the maximum allowable error, "
                         << max_translation_error << std::endl;
                } else {
                    std::cout << "Your yaw error, " << cum_mean_error[2] << " is larger than the maximum allowable error, "
                         << max_yaw_error << std::endl;
                }
                return -1;
            }
        }
    }
	
	// Output the runtime for the filter.
    // Выводим время работы фильтра
	int stop = static_cast<int>(clock());
	double runtime = (stop - start) / double(CLOCKS_PER_SEC);
	std::cout << "Runtime (sec): " << runtime << std::endl;
	
	// Print success if accuracy and runtime are sufficient
    // (and this isn't just the starter code).
    // Выводим сообщение об успешном завершении работы,
    // если фильтр инициализирован и время работы не превышает заданное ограничение;
	if (runtime < max_runtime && pf.initialized()) {
		std::cout << "Success! Your particle filter passed!" << std::endl;
	}
	else if (!pf.initialized()) {
		std::cout << "This is the starter code. You haven't initialized your filter." << std::endl;
	}
	else {
		std::cout << "Your runtime " << runtime << " is larger than the maximum allowable runtime, " << max_runtime << std::endl;
		return -1;
	}
	
	return 0;
}
