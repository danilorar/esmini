// take true distance as input and return perceived distance with noise based on the current mode (IDEAL or NOISY)

#include "PerceptionModel.hpp"
#include <algorithm> // for std::max

namespace scenarioengine 
{
    // constructor
    PerceptionModel::PerceptionModel(unsigned int seed):
    // member initialize
    mode_(Mode::IDEAL), // initial value for mode = IDEAL
    random_generator_(seed) // seed
    {
        // no addition info
    }

  
    // SELECT MODE TYPE
    // function to get the mode and takes input a mode with Mode type (IDEAL/NOISY)
    void PerceptionModel::SetMode(Mode mode){
        mode_ = mode;
    }

    PerceptionModel::Mode PerceptionModel::GetMode() const{
        return mode_;
    }

    // model.SetMode(PerceptionModel::Mode::NOISY);


    // Select ERROR BIN for mode NOISY for an input true_distance 
    // true_distance = relative distance between detect object and ego 
    const PerceptionModel::DistanceErrorBin&
    PerceptionModel::SelectErrorBin(double true_distance) const 
    {

        // Define the error bins for different distance ranges
        static constexpr DistanceErrorBin error_bins[] = {
            {0.0, 25.0, 0.0, 0.10}, // min_distance, max_distance, mean_error, standard_deviation
            {25.0, 50.0, 0.0, 0.30},
            {50.0, 75.0, 0.0, 0.80},
            {75.0, 100.0, 0.0, 1.50},
        };

        // error_bins is like a list of bins, each bin has a min_distance, max_distance, mean_error, and standard_deviation.

        for (const DistanceErrorBin& bin : error_bins) // for each bin in error_bins 
        {
            if (true_distance >= bin.min_distance && true_distance < bin.max_distance) // min_distance <= true_distance < max_distance
            {
                return bin;  // for delta_x return the bin that matches the true_distance
            }
        }

        return error_bins[3]; // If no bin matches, return the last bin (75.0 to 100.0) as fallback
    }


    DistanceMeasurement
    PerceptionModel::MeasureDistance(double true_distance)
    {
        // Distance Measurement is a struct (true_distance, perceived_distance, distance_error)

        // Start each measurement with zero error; add noise below if mode is NOISY.
        DistanceMeasurement measurement{
            true_distance, 
            true_distance,
            0.0,
        };

        // mode stores (NOISY/IDEAL ) 
        if (mode_ == Mode::IDEAL)
        {
            return measurement;
        }

        // based on true_distance = ego - target (euclidean) -> select a error bin
        const DistanceErrorBin& bin = SelectErrorBin(true_distance);

        // create a gaussian distribution by taking the selected bin (previously selected based on true_distance) 
        // from selected bin, take min and std_dev as arguments to build the distribution
        std::normal_distribution<double> distribution( 
            bin.mean_error, 
            bin.standard_deviation
        ); 

        // error
        const double sample_error = distribution(random_generator_);

        // update values in the struct
        measurement.perceived_distance = 
            std::max(0.0, true_distance + sample_error);


        measurement.distance_error = 
            measurement.perceived_distance - true_distance;

        return measurement;

    }
}

// output of measurement (struct) for exm: 
// true_distance = 40.00
// perceived_distance = 39.82
//  distance_error = -0.18
