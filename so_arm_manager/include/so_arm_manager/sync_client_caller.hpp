#ifndef SYNC_SERVICE_CALLER_HPP
#define SYNC_SERVICE_CALLER_HPP

#include "rclcpp/rclcpp.hpp"

// Class to call services in a sync way
template<class ServiceT>
class SyncClient {
    using RequestT = typename ServiceT::Request;
    using ResponseT = typename ServiceT::Response;
    using RequestTSharedPtr = std::shared_ptr<RequestT>;
    using ResponseTSharedPtr = std::shared_ptr<ResponseT>;
public:
    using SharedPtr = std::shared_ptr<SyncClient>;

public:

    SyncClient(
        const rclcpp::Node::SharedPtr node,
        const std::string & service_topic,
        const rclcpp::CallbackGroup::SharedPtr callback_group = nullptr,
        const double timeout = 1.0, // s
        const rmw_qos_profile_t qos = rmw_qos_profile_services_default
    ) :
        node_(node),
        logger_(node->get_logger()),
        service_timeout_(timeout)
    {
        client_ = node->create_client<ServiceT>(service_topic, qos, callback_group);
    }

    const ResponseTSharedPtr call(const RequestTSharedPtr request) const {
        
        const std::string service_name = client_->get_service_name();
        if (!client_->wait_for_service(service_timeout_)) {
            RCLCPP_ERROR_STREAM(
                logger_, 
                "Failed to find service '" << service_name << "'. Is the service name correct?");
            return nullptr;
        }
        
        auto result = client_->async_send_request(request);
        const std::future_status service_status = result.wait_for(service_timeout_);
        if (service_status != std::future_status::ready) {
            RCLCPP_ERROR_STREAM(
                logger_, "Failed to call service '" << service_name
                << "'. Invalid status '" << status2String(service_status) << "' received.");
            return nullptr;
        }
        const auto response_ptr = result.get();

        RCLCPP_ERROR_STREAM_EXPRESSION(
            logger_, response_ptr == nullptr, 
            "Failed to call service '" << service_name << "'. Null result received.");

        return response_ptr;
    }

    const ResponseTSharedPtr call(const RequestT &request) const {
        // Cria o shared ptr internamente por conveniência
        return call(std::make_shared<RequestT>(request));
    }

    bool call(const RequestT & request, ResponseT & response) const {
        auto response_ptr = call(request);
        if (response_ptr == nullptr) {return false;}
        response = *response_ptr;
        return true;
    }

    bool call(const RequestTSharedPtr request, ResponseTSharedPtr response) const {
        response = call(request);
        if (response == nullptr) {return false;}
        return true;
    }

private:
    std::string status2String(const std::future_status & service_status) const {

        if (service_status == std::future_status::ready) {
            return "ready";
        }
        if (service_status == std::future_status::deferred) {
            return "deferred";
        }
        if (service_status == std::future_status::timeout) {
            return "timeout";
        }
        return "unknow";
    }

    rclcpp::Node::SharedPtr node_;
    typename rclcpp::Client<ServiceT>::SharedPtr client_;
    const rclcpp::Logger logger_;

    std::chrono::duration<double> service_timeout_;
};

#endif  // SYNC_SERVICE_CALLER_HPP
