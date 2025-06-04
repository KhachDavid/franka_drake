#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

/** 
 A “mask” system that takes a 7×1 input vector (τ_gc from InverseDynamics) 
 and passes it straight through into its 7×1 output, except that the last entry 
 is always overwritten with zero. 
*/
class ZeroLastJoint : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ZeroLastJoint);

  ZeroLastJoint() {
    // One input port: a 7-vector (gravity-comp torque)
    this->DeclareVectorInputPort("tau_in", BasicVector<double>(7));
    // One output port: a 7-vector (modified gravity-comp torque)
    this->DeclareVectorOutputPort(
        "tau_out",
        BasicVector<double>(7),
        &ZeroLastJoint::CopyAndZeroLast);
  }

 private:
  // Cache callback: Simply copy in[0..5] → out[0..5], then out[6] = 0.
  void CopyAndZeroLast(const Context<double>& context,
                       BasicVector<double>* out) const {
    const auto& in = this->get_input_port(0).Eval(context);
    // Sanity check that input really is length 7:
    DRAKE_DEMAND(in.size() == 7);
    auto y = out->get_mutable_value();
    for (int i = 0; i < 6; ++i) {
      y(i) = in(i);
    }
    y(6) = 0.0;  // force the 7th entry to zero
  }
};

}  // namespace systems
}  // namespace drake
