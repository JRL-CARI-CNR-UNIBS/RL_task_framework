#ifndef XML_FUNCTIONS_H
#define XML_FUNCTIONS_H

namespace skills_util
{
    std::vector<std::string> getMemberByXml(const XmlRpc::XmlRpcValue xml_file);

    template<typename T>
    inline bool getParam(const std::string param_ns, const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value)
    {
        std::string param_str = "/"+param_ns+"/"+action_name+"/"+skill_name+"/"+param_name;
        if ( !n_.getParam(param_str, param_value) )
        {
            return false;
        }
        return true;
    }

    template<typename T>
    inline bool getParam(const std::string param_ns, const std::string &action_name, const std::string &param_name, T &param_value)
    {
        std::string param_str = "/"+param_ns+"/"+action_name+"/"+param_name;
        if ( !n_.getParam(param_str, param_value) )
        {
            return false;
        }
        return true;
    }

    template<typename T>
    inline void setParam(const std::string param_ns, const std::string &action_name, const std::string &skill_name, const std::string &param_name, const T &param_value)
    {
        std::string param_str = "/"+param_ns+"/"+action_name+"/"+skill_name+"/"+param_name;

        n_.setParam(param_str, param_value);
        return;
    }

    template<typename T>
    inline void setParam(const std::string param_ns, const std::string &action_name, const std::string &param_name, const T &param_value)
    {
        std::string param_str = "/"+param_ns+"/"+action_name+"/"+param_name;

        n_.setParam(param_str, param_value);
        return;
    }

} // end namespace skills_util


#endif // XML_FUNCTIONS_H
