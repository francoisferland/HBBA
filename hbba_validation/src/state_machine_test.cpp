/// \file state_machine.cpp Contains a test implementation of a state machine.
#include <hbba_validation/state_machine.hpp>
#include <iostream>

using namespace hbba_validation;

namespace {
    class Host
    {
    private:
        typedef StateMachine<Host> SM;
        SM sm_;

        enum {
            STATE_A = 0,
            STATE_B,
            STATE_C,
            STATE_SIZE
        };

        enum {
            EVENT_0 = 0,
            EVENT_1,
            EVENT_SIZE
        };

    private:
        SM::Handle stateA()
        {
            std::cerr << "In state A." << std::endl;
            return STATE_A;
        }

        SM::Handle stateB()
        {
            std::cerr << "In state B." << std::endl;
            return STATE_C;
        }

        SM::Handle stateC()
        {
            std::cerr << "In state C." << std::endl;
            return STATE_C;
        }

    public:
        Host()
        {
            SM::Transitions transitions;
            SM::generateTransitionsMatrix(STATE_SIZE, 
                                          EVENT_SIZE, 
                                          transitions);

            SM::States states;
            states.push_back(&Host::stateA);
            states.push_back(&Host::stateB);
            states.push_back(&Host::stateC);

            transitions[STATE_A][EVENT_0] = STATE_B;
            transitions[STATE_C][EVENT_1] = STATE_A;

            sm_ = SM(states, transitions, this);
        }

        void run()
        {
            // Shoud print: A, B, C, A.
            sm_.pushEvent(EVENT_1);
            sm_.pushEvent(EVENT_0);
            sm_.processQueue();
            sm_.pushEvent(EVENT_1);
            sm_.processQueue();
            sm_.pushEvent(EVENT_1);
            sm_.processQueue();
            sm_.processQueue();
        }

    };

}

int main(int argc, char** argv)
{
    Host host;
    host.run();
}

