classdef OHValkyrie < Valkyrie
  methods
    function obj = OHValkyrie(urdf, options)

      if nargin < 2
        options = struct();
      end
      options = applyDefaults(options,...
                              struct('use_new_kinsol', true));

      if nargin < 1 || isempty(urdf)
        urdf = strcat(getenv('DRC_PATH'),'/models/val_description/urdf/valkyrie_sim_drake.urdf');
      else
        typecheck(urdf,'char');
      end

      if ~isfield(options,'hands')
        options.hands = 'none';
      end

      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');

      obj = obj@Valkyrie(urdf, options);

      warning(S);
    end

    function obj = compile(obj)
      S = warning('off','Drake:RigidBodyManipulator:SingularH');
      obj = compile@TimeSteppingRigidBodyManipulator(obj);
      warning(S);

      state_frame = drcFrames.ValkyrieState(obj);
      obj = obj.setStateFrame(state_frame);
      obj = obj.setOutputFrame(state_frame);

      % input_frame = OHValkyrieInput(obj);
      % obj = obj.setInputFrame(input_frame);
    end
  end
end

