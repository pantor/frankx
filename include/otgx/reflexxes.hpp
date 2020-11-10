#pragma once

#include <Eigen/Core>

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include <otgx/parameter.hpp>


namespace otgx {

template<size_t DOFs>
class Reflexxes {
    InputParameter<DOFs> current_input;
    std::shared_ptr<ReflexxesAPI> rml;
    std::shared_ptr<RMLPositionInputParameters> input_parameters;
    std::shared_ptr<RMLPositionOutputParameters> output_parameters;

    int result_value = 0;
    RMLPositionFlags flags;

public:
    double delta_time;

    explicit Reflexxes(double delta_time): delta_time(delta_time) {
        rml = std::make_shared<ReflexxesAPI>(DOFs, delta_time);
        input_parameters = std::make_shared<RMLPositionInputParameters>(DOFs);
        output_parameters = std::make_shared<RMLPositionOutputParameters>(DOFs);

        flags.SynchronizationBehavior = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
    }

    Result update(const InputParameter<DOFs>& input, OutputParameter<DOFs>& output) {
        if (input != current_input) {
            current_input = input;

            for (size_t i = 0; i < DOFs; i += 1) {
                input_parameters->SelectionVector->VecData[i] = true;
                input_parameters->CurrentPositionVector->VecData[i] = input.current_position(i);
                input_parameters->CurrentVelocityVector->VecData[i] = input.current_velocity(i);
                input_parameters->CurrentAccelerationVector->VecData[i] = input.current_acceleration(i);
                input_parameters->TargetPositionVector->VecData[i] = input.target_position(i);
                input_parameters->TargetVelocityVector->VecData[i] = input.target_velocity(i);
                input_parameters->MaxVelocityVector->VecData[i] = input.max_velocity(i);
                input_parameters->MaxAccelerationVector->VecData[i] = input.max_acceleration(i);
                input_parameters->MaxJerkVector->VecData[i] = input.max_jerk(i);
            }
        }

        result_value = rml->RMLPosition(*input_parameters, output_parameters.get(), flags);

        for (size_t i = 0; i < DOFs; i += 1) {
            output.new_position(i) = output_parameters->NewPositionVector->VecData[i];
            output.new_velocity(i) = output_parameters->NewVelocityVector->VecData[i];
            output.new_acceleration(i) = output_parameters->NewAccelerationVector->VecData[i];
        }

        if (result_value == ReflexxesAPI::RML_FINAL_STATE_REACHED) {
            return Result::Finished;
        } else if (result_value == ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES) {
            return Result::Error;
        }
        return Result::Working;
    }
};

} // namespace otgx
