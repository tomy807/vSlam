#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "common_include.h" 

namespace myslam {
    class Config {
        private:
            static std::shared_ptr<Config> config_; 
            cv::FileStorage file_;
            
            Config () {
            } 
        public:
            ~Config();
            
            static void setParameterFile( const std::string& filename ); 
            
            template< typename T >
            static T get( const std::string& key )
            {
                T result;
                Config::config_->file_[key] >> result;
                return result;                
            }
    };
}

#endif // CONFIG_H