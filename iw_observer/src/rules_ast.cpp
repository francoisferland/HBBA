#include <iw_observer/rules_ast.hpp>
#include <iw_observer/runtime.hpp>

using namespace iw_observer;

Rules& iw_observer::ruleset()
{
    static Rules ruleset;
    
    return ruleset;
}

AddCommand::AddCommand(const Ident& des_cls, const Args& args)
{
    static int unique_id = 0;

    std::stringstream id_ss;
    id_ss << "iwo_" << des_cls << "_" << unique_id++;

    desire_.id   = id_ss.str();
    desire_.type = des_cls;

    typedef Args::const_iterator It;
    for (It i = args.begin(); i != args.end(); ++i) {
        const Arg* arg = *i;
        arg->apply(desire_);
    }
}

void AddCommand::exec(Runtime& rt, const hbba_msgs::Event& evt) const
{
    rt.addDesire(desire_);
}

DelCommand::DelCommand(const Ident& des_id): id_(des_id)
{
}

void DelCommand::exec(Runtime& rt, const hbba_msgs::Event& evt) const
{
    // Special case of the command: if the specified desire id is "-",
    // delete the desire that triggered the event instead.
    if (id_ == "-") {
        rt.removeDesire(evt.desire);
    } else {
        rt.removeDesire(id_);
    }
}

