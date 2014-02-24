#ifndef TOUR_GUIDE_HPP
#define TOUR_GUIDE_HPP

#include "state_machine.hpp"
#include <hbba_msgs/Desire.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace hbba_validation
{
    class TourGuide;

    typedef StateMachine<TourGuide> TourGuideStateMachine;

    /// \brief An abstract class that represent a single tour step.
    class TourStep
    {
    public:
        typedef TourGuideStateMachine::Handle StateHandle;

    private:
        std::string name_;
        StateHandle handle_;
        StateHandle next_;

    public:
        /// \brief Constructor.
        ///
        /// \param name Name of the tour step, publicly available through
        ///             name().
        /// \param 
        TourStep(const std::string& name): name_(name) {}

        virtual ~TourStep() {}

        /// \brief Return the name of the step.
        const std::string& name() { return name_; }

        /// \brief Set this step's state machine handle.
        void handle(const StateHandle& h) { handle_ = h; }
        const StateHandle& handle() const { return handle_; }

        /// \brief Set the state  machine handle of the step following this one.
        void next(const StateHandle& h) { next_ = h; }
        const StateHandle& next() const { return next_; }

        /// \brief Execute the state and return true if the scenario should
        /// progress to the next state.
        /// 
        /// \param desires_set A reference to a desire set to fill.
        ///                    Desires in this set will be added to the
        ///                    Intention Workspace and marked for deletion when
        ///                    the state ends.
        virtual bool run(std::vector<hbba_msgs::Desire>& desires_set) = 0;

    };

    /// \brief A state machine-based motivation module for guided tours.
    ///
    /// The scenario is described on the parameter server as YAML and is built 
    /// out of a few types of states/actions:
    ///
    ///  - waypoint:   A pose in the global frame to reach.
    ///                Defined by a landmark (QR-Code) detection location.
    ///                Goes to the next step when successful.
    ///  - speech:     A speech action: read the given text and switch to the 
    ///                next step.
    ///                Cannot fail.
    ///                Optionally point to the left or right of the robot.
    ///  - unlock:     Unlock a door with a cardreader.
    ///                Assumes the cardreader is roughly in front of the robot.
    ///                Goes to the next step when successful.
    ///  - turnaround: Rotate 180 degrees in place, then go to the next step. 
    ///                Useful when facing the audience before a speech is
    ///                required.
    ///                Cannot fail.
    ///
    /// See hbba_validation/data/visit.yaml for a sample setup.
    /// Each state is a single sub-dictionary, and all steps are expected to be
    /// defined in the "~tour" sub-namespace.
    /// A special entry named "start" defines the first step in the scenario.
    /// A special state named "stop" can be specified as the next state to end
    /// the scenario.
    ///                
    /// As the machine mostly interact with HBBA, it only requires access to the
    /// add_desires and remove_desires services, and events topic.
    class TourGuide
    {
    private:
        typedef StateMachine<TourGuide> SM;

        SM                     sm_;
        std::vector<TourStep*> steps_;

    public:
        /// \brief Constructor.
        ///
        /// Parse the scenario defined in the np namespace.
        ///
        /// \param n  Node handle for topics and services.
        /// \param np Node handle for parameters, which should include a "tour"
        ///           sub-namespace.
        TourGuide(ros::NodeHandle& n, ros::NodeHandle& np);

        ~TourGuide();

    private:
        enum {
            EVENT_LOC_REACHED,
            EVENT_SPEECH_DONE,
            EVENT_TURNAROUND_DONE,
            EVENT_UNLOCKED,
            EVENT_SIZE
        };

        bool parseScenario(const ros::NodeHandle& np);
        bool parseElem(const std::string&         key, 
                             XmlRpc::XmlRpcValue& elem,
                             std::string&         next,
                             unsigned int&        type);

        SM::Handle step();
       
    };

    class WaypointStep: public TourStep
    { 
    private:
        geometry_msgs::PoseStamped pose_;

    public:
        WaypointStep(const std::string& name): TourStep(name) {}

        void pose(const geometry_msgs::PoseStamped& pose) { pose_ = pose; }

        bool run(std::vector<hbba_msgs::Desire>& desires_set);
    };

    class SpeechStep: public TourStep
    {
    public:
        enum Point
        {
            POINT_NONE,
            POINT_LEFT,
            POINT_RIGHT
        };

    private:
        std::string text_;
        Point       point_at_;

    public:
        SpeechStep(const std::string& name): 
            TourStep(name), 
            point_at_(POINT_NONE)
        {
        }

        void text(const std::string& text) { text_ = text; }

        void pointAt(Point p) { point_at_ = p; }

        bool run(std::vector<hbba_msgs::Desire>& desires_set);
    };

    class UnlockStep: public TourStep
    {
    public:
        UnlockStep(const std::string& name): TourStep(name) {}

        bool run(std::vector<hbba_msgs::Desire>& desires_set);
    };

    class TurnaroundStep: public TourStep
    {
    public:
        TurnaroundStep(const std::string& name): TourStep(name) {}

        bool run(std::vector<hbba_msgs::Desire>& desires_set);
    };

}

#endif

