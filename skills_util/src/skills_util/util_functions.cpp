/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <skills_util/log.h>

namespace skills_util
{
    std::vector<std::string> getMemberByXml(const XmlRpc::XmlRpcValue xml_file)
    {
        std::string xml_str = xml_file.toXml();
        std::vector<std::string> members;

        int level = 0, index = 0;
        bool end = false;
        while (!end)
        {
            if (xml_str.find("<member>", index) == std::string::npos)
            {
                end = true;
                continue;
            }
            if (xml_str.find("<member>", index) < xml_str.find("</member>", index) )
            {
                level++;
                index = xml_str.find("<member>", index) + 8;
            }
            else
            {
                level--;
                index = xml_str.find("</member>", index) + 9;
                continue;
            }

            if (level == 1)
            {
                int index_start_name = xml_str.find("<name>", index) + 6;
                int name_lenght = xml_str.find("</name>", index) - index_start_name;
                members.push_back( xml_str.substr(index_start_name, name_lenght) );
            }
        }
        return members;
    }
}
