function setup_ad_build()
base_dir = 'C:\Users\robertph\Desktop\HeliControl\matlab\autodiff\';

coder.updateBuildInfo('addIncludePaths', [base_dir 'include'])
coder.updateBuildInfo('addLinkObjects', 'heli_adolc.lib', [base_dir 'lib'], '', true, true)
coder.updateBuildInfo('addLinkObjects', 'adolc.lib', [base_dir 'lib'], '', true, true)
end

