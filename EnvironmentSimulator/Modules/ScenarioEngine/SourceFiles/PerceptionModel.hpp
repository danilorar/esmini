#pragma once // to prevent multiple inclusions of this header file

#include <random> // seed random number generator

namespace scenarioengine{

    // Create a struct to store distance measurement data
    struct DistanceMeasurement{
        double true_distance;
        double perceived_distance;
        double distance_error;
    };


    // Create a class to represent the perception model
    class PerceptionModel{

        public:

            /* It's like when you define double or int,
            but here we define a new type called Mode with two possible values: IDEAL and NOISY*/
            enum class Mode
            {  // 
                IDEAL, 
                NOISY
            };

            explicit PerceptionModel(unsigned int seed = 42);

            // Function to set the perception model mode 
            void SetMode(Mode mode); 

            Mode GetMode() const; // Function to get the current perception model mode

            
            // This function is used to measure the distance to an object, and it takes the true distance as input. Depending on the current mode
            // (IDEAL or NOISY), it will either return the true distance (in IDEAL mode) or a perceived distance with some noise added (in NOISY
            // mode). The function returns a DistanceMeasurement struct that contains the true distance, perceived distance, and the error between
            // them.
            DistanceMeasurement MeasureDistance(double true_distance);

        private:
            struct DistanceErrorBin
            {
                double min_distance;
                double max_distance;
                double mean_error;
                double standard_deviation;
            };

            // Function to select the appropriate error bin based on the true distance
            const DistanceErrorBin& SelectErrorBin(double true_distance) const;

            Mode mode_; 
            std::mt19937 random_generator_; // Random number generator

        };
}

