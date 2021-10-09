#
# Cookbook:: online
# Recipe:: webnode2
#
# Copyright:: 2021, The Authors, All Rights Reserved.

package	'httpd' do
action	:install
end

service 'httpd' do
action [:enable, :start]
end

file '/var/www/html/index.html' do
content 'Welcome ji! Ki haal twada?'
action :create
end
