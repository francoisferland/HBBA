#ifndef LANDMARKS_INTERACTION_HPP
#define LANDMARKS_INTERACTION_HPP

#include "goto_landmark.hpp"
#include "state_machine.hpp"
#include <boost/range.hpp>
#include <string>
namespace hbba_validation
{
    /// \brief Motivation module for interacting with people about landmarks.
    ///
    /// The goal is to respond to basic requests to go to landmarks mostly 
    /// identified by QR codes.
    /// All landmarks are identified by a single number.
    /// The QR code format is "IRL1_LANDMARK_X". 
    ///
    /// Two direct requests are implemented:
    ///
    ///  - "This is landmark X": instructs the LandmarksObserver to store the 
    ///    current location as landmark X.
    ///    Note that this request can overwrite landmarks discovered by QR
    ///    codes, and vice versa.
    ///  - "Go to landmark X": Go to the specified landmark, or tell the person
    ///    that the robot doesn't know this landmark.
    ///    In that case, the person can lead the robot to the landmark and make
    ///    it read the QR code or use the first request.
    ///
    /// Topics (besides those used by GoToLandmarks):
    ///
    ///  - sphinx_result: Speech recognition output.
    ///
    class LandmarksInteraction
    {
    private:
        ros::Subscriber sub_sphinx_;
        GoToLandmark    goto_;

        std::string last_code_;     // Last landmark identifier received.
        std::string last_code_num_; // Last landmark identifier, numerical part.

        // State identifiers:
        enum {
            STATE_WAIT = 0,     // Initial state.
            STATE_SAVE,         // Save the current location as a landmark.
            STATE_LEAD,         // The robot is being lead to a landmark.
            STATE_GOTO,         // The robot is going to a known landmark.
            STATE_SIZE
        };

        // Event identifiers:
        enum {
            EVENT_REQ_SAVE = 0, // A request to save a landmark was received.
            EVENT_REQ_GOTO,     // A request to go to a landmark was received.
            EVENT_SIZE
        };

        typedef StateMachine<LandmarksInteraction> SM;
        SM sm_;

        typedef boost::iterator_range<std::string::const_iterator> StringRange;

    public:
        /// \brief Constructor.
        ///
        /// \param n  Node handle for topics.
        /// \param np Node handle for parameters.
        LandmarksInteraction(ros::NodeHandle& n, ros::NodeHandle& np);

    private:
        SM::Handle stateWait();
        SM::Handle stateSave();
        SM::Handle stateLead();
        SM::Handle stateGoTo();

        void newLandmarkCB(const std::string& code);

        /// \brief Push an event to the state machine queue and automatically
        /// process it.
        void pushEvent(const SM::Handle event);

        void sphinxCB(const std_msgs::String& msg);

        /// \brief Take a single number string and convert it in a valid
        /// QR-style landmark code.
        ///
        /// The process is limited to adding a prefix to the number.
        std::string formatLandmark(const std::string& code) const;

        /// \brief Extract a landmark code from a request.
        /// 
        /// \param req Original request string.
        /// \param loc Location where the request was found.
        ///            The landmark code should be situated right after this
        ///            location.
        /// \param out Output string, fully formatted ("IRL_LANDMARK_...").
        /// \param num Landmark identifier output, numerical part only.
        ///
        /// \return False if the code could not be extracted, usually because
        ///               the string is too short.
        ///               The output string will not be affected.
        bool codeFromRequest(const std::string& req, 
                             const StringRange& loc,
                             std::string&       out,
                             std::string&       num) const;

        /// \brief Produces a desire to say something.
        ///
        /// \param text Text to say.
        void say(const std::string& text);
    };
}

#endif

