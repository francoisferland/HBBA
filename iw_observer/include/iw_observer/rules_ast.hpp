#ifndef RULES_AST_HPP
#define RULES_AST_HPP

#include <hbba_msgs/Desire.h>
#include <string>
#include <vector>
#include <iostream>

namespace iw_observer
{

    // Forward declaration:
    class Runtime;

    /// \brief Underlying type for identifiers (a basic string).
    typedef std::string Ident;

    /// \brief Base class for arguments.
    ///
    /// The parser creates instances of classes derived from this one.
    /// Instances of the Command class then call apply() on the desire the 
    /// command refers to.
    struct Arg
    {
        Arg() { }
        virtual ~Arg() {}

        /// \brief Applies the value of the argument to the given desire.
        /// 
        /// Needs to be implemented in the derived class.
        /// For instance, an utility argument class would set the desire's 
        /// intensity to its saved value.
        virtual void apply(hbba_msgs::Desire& d) const = 0;
    };


    /// \brief Desire id argument.
    struct IdArg: public Arg
    {
    private:
        Ident value_;

    public:
        IdArg(const Ident& v): value_(v) { std::cerr << "New id arg, v: " << v << std::endl; }
        void apply(hbba_msgs::Desire& d) const { d.id = value_; }
    };
    /// \brief Desire utility argument.
    struct UtilArg: public Arg
    {
    private:
        int value_;

    public:
        UtilArg(int v): value_(v) { std::cerr << "New util arg, v: " << v << std::endl; }
        void apply(hbba_msgs::Desire& d) const { d.utility = value_; }
    };

    /// \brief Desire intensity argument.
    struct IntArg: public Arg
    {
    private:
        int value_;

    public:
        IntArg(int v): value_(v) {}
        void apply(hbba_msgs::Desire& d) const { d.intensity = value_; }
    };

    /// \brief Desire parameters argument.
    ///
    /// Does not parse the arguments, just pass the string as a whole, as it's
    /// supposed to be formatted as a YAML string.
    struct ParamsArg: public Arg
    {
    private:
        std::string value_;

    public:
        ParamsArg(const std::string& v): value_(v) {}
        void apply(hbba_msgs::Desire& d) const { d.params = value_; }
    };

    typedef std::vector<Arg*> Args;

    /// \brief Generic class for rules.
    struct Rule
    {
        Rule() { std::cerr << "New rule added" << std::endl; }
    };

    typedef std::vector<Rule*> Rules;

    /// \brief Return a reference to the current ruleset.
    ///
    /// Used by the parser and will contain its output after yyparse().
    /// Not thread-safe.
    Rules& ruleset();

    /// \brief Base class for commands.
    ///
    /// Commands are usually applied to a desire, but the exact type of the 
    /// command specifies how this desire is managed.
    /// For instance, a delete command only require the id of the desire, not a 
    /// full description of it.
    /// The parser generates instances of classes derived from this one.
    /// When executed with the exec() function, commands are given a reference 
    /// to the IWObserver runtime.
    class Command
    {
    public:
        Command() {}
        virtual ~Command() {}

        /// \brief Execute the command.
        ///
        /// Must be implemented in derived classes.
        ///
        /// \param rt A mutable reference to the IWObserver runtime.
        virtual void exec(Runtime& rt) = 0;
    };

    /// \brief Command class to create and add a new desire to the Intention
    /// Workspace.
    class AddCommand: public Command
    {
    private:
        hbba_msgs::Desire desire_;
        
    public:
        /// \brief Constructor.
        ///
        /// The default id for the created the desire will be of the form
        /// "iwo_CLASS_ID"
        /// The ID part is a simple integer incremented at each call, making
        /// this constructor not thread safe.
        /// Arguments are applied at construction time.
        ///
        /// \param des_cls Single identifier that refers to the desire class to
        ///                create.
        /// \param args    Arguments to apply to the new desire (see Arg class).
        AddCommand(const Ident& des_cls, const Args& args);

        void exec(Runtime& rt);
    };

    /// \brief Command class to delete an already existing desire from the
    /// Intention Workspace.
    /// 
    /// Does not perform any test if the desire does not exists - transfers this
    /// responsability to the Intention Workspace node.
    class DelCommand: public Command
    {
    private:
        const std::string id_;

    public:
        /// \brief Constructor.
        ///
        /// \param des_id Identifier of the desire to delete.
        DelCommand(const Ident& des_id);

        void exec(Runtime& rt);
    };

    typedef std::vector<Command*> Commands;

}

#endif

