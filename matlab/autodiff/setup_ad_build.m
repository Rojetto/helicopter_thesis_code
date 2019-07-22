function setup_ad_build()
persistent is_setup

if isempty(is_setup)
    is_setup = false;
end

if ~is_setup
    base_dir = 'C:\dev\HeliControl\matlab\autodiff\';
    
    coder.updateBuildInfo('addIncludePaths', [base_dir 'include'])
    coder.updateBuildInfo('addLinkObjects', 'heli_adolc.lib', [base_dir 'lib'], '', true, true)
    coder.updateBuildInfo('addLinkObjects', 'adolc.lib', [base_dir 'lib'], '', true, true)
    
    is_setup = true;
end
end

