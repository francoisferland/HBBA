#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include <vector>
#include <queue>
#include <map>
#include <cassert>

namespace hbba_validation
{
    /// \brief Basic state transition machine template for motivation modules.
    ///
    /// This class is meant to be used in (and with) an instance of class T,
    /// called the host.
    /// The host generates a state vector, which are pairs of numerical ids (see 
    /// Handle typedef) and a function pointer to a T class method with no 
    /// parameters. 
    /// It also generates a transitions matrix, which maps event ids to future
    /// state ids, depending on the current state id.
    /// Each row represents the current state, each column the event id, and
    /// values are the next state ids.
    /// To indicate a non-transition, the value should be equal to the the
    /// current's state id (the matrix' row).
    /// To assist with the creation of these matrices, see the static
    /// generateEventsMatrix() function.
    ///
    /// The host class then generates events, which are pushed to the machine's
    /// queue (with pushEvent()).
    /// When the host wants to cycle through the states, it can call the
    /// processQueue() function, which evaluates each event on the queue and 
    /// calls the appropriate functions for each transition.
    /// A state function is only called on a valid transition, see the 
    /// processQueue() function for details.
    /// 
    /// State functions have to return the next state id when they finish
    /// executing.
    /// They can of course return their current event if no transition should
    /// occur.
    ///
    /// A good practice is to have two enums for state and events ids.
    /// This ensures that ids will be sequential and are compatible with this
    /// class' handles (unsigned int).
    template <class T>
    class StateMachine
    {
    public:
        /// \brief Numeric handle used by events and states.
        typedef unsigned int Handle;

        typedef Handle (T::*StateFunction)();
        typedef std::vector<StateFunction> States;

        typedef std::vector<Handle>            TransitionsVector;
        typedef std::vector<TransitionsVector> Transitions;

        typedef std::queue<Handle> EventsQueue;

    private:
        States       states_;       // State functions.
        Transitions  transitions_;  // Transitions matrix.
        T*           host_;         // Host instance.
        Handle       events_count_; // Number of event ids.
        EventsQueue  events_queue_; // Events queue.
        Handle       state_;        // Current state.

    public:
        /// \brief Generates an event transitions matrix.
        /// 
        /// The matrix is filled in row-major order.
        /// Each row represents the current state id, each column the event id,
        /// and the values are the next state id in the transition.
        /// The matrix will be filled with null transitions, i.e. the current
        /// state's id, or row index, which means only relevant transitions will
        /// need to be filled.
        ///
        /// \param sc  The number of states, or one pass the last state id.
        /// \param ec  The number of events, or one pass the last event id.
        /// \param mtx A reference to the output matrix.
        static void generateTransitionsMatrix(Handle sc, 
                                              Handle ec, 
                                              Transitions& mtx)
        {
            mtx.resize(sc);
            for (Handle i = 0; i < sc; ++i) {
                mtx[i] = TransitionsVector(ec, i);
            }
        }

        /// \brief Default constructor: does not produce a working state
        /// machine.
        StateMachine():
            state_(0)
        {}

        /// \brief Constructor.
        ///
        /// The machine will be initialized in the first state (handle 0).
        /// This state's function is always called (if not null) at the end of
        /// this constructor, as are succeeding states if the first state handle
        /// return a different state.
        /// It is recommanded however that this state should only be used for
        /// intialization purposes.
        /// 
        /// \param states      The vector of state functions.
        /// \param transitions The transitions matrix, see
        ///                    generateTransitionsMatrix on how to fill it.
        /// \param host        A pointer to an instance of the host class.
        StateMachine(const States&      states, 
                     const Transitions& transitions,
                     T*                 host):
            states_(states),
            transitions_(transitions),
            host_(host),
            state_(0)
        {
            // A few sanity checks: if we have at least one state, if the row
            // count in the transitions matrix is correct, and if all columns
            // are the same size.

            assert(states_.size() > 0);
            assert(states_.size() == transitions_.size());
            
            events_count_ = transitions_.begin()->size();

            for (Handle i = 0; i < transitions_.size(); ++i) {
                assert(transitions_[i].size() == events_count_);
            }

            if (states_[0] != 0) {
                Handle last_state = state_;
                state_ = callState(state_);
                while (state_ != last_state) {
                    last_state = state_;
                    state_     = callState(state_);
                }
            }
                
        }

        /// \brief Return the current state of the machine.
        Handle state() const { return state_; }

        /// \brief Return the number of states managed by this machine.
        Handle size() const { return states_.size(); }

        /// \brief Return the number of possible events.
        Handle eventsCount() const { return events_count_; }

        /// \brief Add an event to end of the queue.
        void pushEvent(const Handle h)
        {
            assert(h < eventsCount());
            events_queue_.push(h);
        }

        /// \brief Flush the event queue, cycle through the states.
        /// 
        /// Here is the process:
        ///   while event queue is not empty:
        ///     last_state = state
        ///     state = transitions[state][event];
        ///     while (state != last_state):
        ///         last_state = state
        ///         state = call(state)
        ///     
        void processQueue()
        {
            while (!events_queue_.empty()) {
                Handle event      = events_queue_.front();
                Handle last_state = state_;
                state_ = transitions_[state_][event];
                while (state_ != last_state) {
                    last_state = state_;
                    state_     = callState(state_);
                }
                events_queue_.pop();
            }
        }

    private:
        /// \brief Call a state function, return with the requested next state.
        ///
        /// \return The next state handle if the function pointer is valid, or
        ///         the current state if it isn't.
        Handle callState(const Handle state)
        {
            assert(state < size());
            const StateFunction& fun = states_[state];
            if (fun != 0) {
                return (host_->*fun)();
            } else {
                return state_;
            }
        }
    };
}

#endif

