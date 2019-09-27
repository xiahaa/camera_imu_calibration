classdef zncc
    methods(Static)
        function g = gaussian_kernel(gstd)
            % """Generate odd sized truncated Gaussian
            %
            % The generated filter kernel has a cutoff at $3\sigma$
            % and is normalized to sum to 1
            %
            % Parameters
            % -------------
            % gstd : float
            %         Standard deviation of filter
            %
            % Returns
            % -------------
            % g : ndarray
            %         Array with kernel coefficients
            % """
            Nc = ceil(gstd*3)*2+1;
            x = linspace(-(Nc-1)/2,(Nc-1)/2,Nc);
            g = exp(-0.5*((x/gstd).^2));
            g = g./sum(g);
        end
        
        function ts_out = subsample(time_series, downsample_factor)
            % """Subsample with Gaussian prefilter
            %
            % The prefilter will have the filter size $\sigma_g=.5*ssfactor$
            %
            % Parameters
            % --------------
            % time_series : ndarray
            %         Input signal
            % downsample_factor : float
            %         Downsampling factor
            %
            % Returns
            % --------------
            %    ts_out : ndarray
            %         The downsampled signal
            % """
            Ns = int(floor(length(time_series)/downsample_factor));
            g = gaussian_kernel(0.5*downsample_factor);
            ts_blur = convolve(time_series,g,'same');
            ts_out = zeros(1,Ns);
            for k = 1:length(Ns)
                cpos  = (k+.5)*downsample_factor-.5;
                cfrac = cpos-floor(cpos);
                cind  = max(floor(cpos),1);
                if cfrac>0
                    ts_out(k)=ts_blur(cind)*(1-cfrac)+ts_blur(cind+1)*cfrac;
                else
                    ts_out(k)=ts_blur(cind);
                end
            end
        end
        
        function ts_out = upsample(time_series, scaling_factor)
            % """Upsample using linear interpolation
            %
            % The function uses replication of the value at edges
            %
            % Parameters
            % --------------
            % time_series : ndarray
            %         Input signal
            % scaling_factor : float
            %         The factor to upsample with
            %
            % Returns
            % --------------
            % ts_out  : ndarray
            %         The upsampled signal
            % """
            Ns0 = length(time_series);
            Ns  = int32(floor(Ns0*scaling_factor));
            ts_out = zeros(Ns,1);
            for k = 1:Ns
                cpos  = int32(min([Ns0-1,max([0.,(k+0.5)/scaling_factor-0.5])]));
                cfrac = cpos-floor(cpos);
                cind  = max(1,int32(floor(cpos)));
                if cfrac>0
                    ts_out(k)=time_series(cind)*(1-cfrac)+time_series(cind+1)*cfrac;
                else
                    ts_out(k)=time_series(cind);
                end
            end
        end
        
        function ts_out = do_binning(time_series,factor)
            Ns = int(length(time_series) / factor);
            ts_out = zeros(Ns,1);
            for k = 1:Ns
                ts_out(k)=0;
                for l = 1:factor
                    ts_out(k) = ts_out(k) + time_series(k*factor+l);
                end
                ts_out(k) = ts_out(k)/factor;
            end
        end
        
        function pyr_out = create_pyramid(time_series,octaves)
            pyr_out = {time_series};
            for k = 1:octaves
                pyr_out{end+1}=(do_binning(pyr_out{end},2));
            end
        end
        
        function [best_shift,ts_out] = zncc(ts1,ts2)
            % """Zero mean normalised cross-correlation (ZNCC)
            %
            % This function does ZNCC of two signals, ts1 and ts2
            % Normalisation by very small values is avoided by doing
            % max(nmin,nvalue)
            %
            % Parameters
            % --------------
            % ts1 : ndarray
            %         Input signal 1 to be aligned with
            % ts2 : ndarray
            %         Input signal 2
            %
            % Returns
            % --------------
            % best_shift : float
            %         The best shift of *ts1* to align it with *ts2*
            % ts_out : ndarray
            %         The correlation result
            % """
            % Output is the same size as ts1
            Ns1 = length(ts1);
            Ns2 = length(ts2);
            ts_out = zeros(Ns1,1);
            ishift = int(floor(Ns2/2)); % origin of ts2
            
            t1m = mean(ts1);
            t2m = mean(ts2);
            
            for k = 1:Ns1
                lstart = int(ishift-k)
                if lstart< 1
                    lstart=1;
                end
                lend = int(ishift-k+Ns2);
                imax = int(min([Ns2,Ns1-k+ishift]));
                if lend>imax 
                    lend=imax
                end
                csum = 0;
                ts1sum = 0;
                ts1sum2 = 0;
                ts2sum = 0;
                ts2sum2 = 0;
                
                Nterms = lend-lstart;
                for l = lstart:lend
                    csum    = csum+ts1(k+l-ishift)*ts2(l);
                    ts1sum  = ts1sum+ts1(k+l-ishift);
                    ts1sum2 = ts1sum2+ts1(k+l-ishift)*ts1(k+l-ishift);
                    ts2sum  = ts2sum+ts2(l);
                    ts2sum2 = ts2sum2+ts2(l)*ts2(l);
                end
                ts1sum2 = max([t1m*t1m*100,ts1sum2])-ts1sum*ts1sum/Nterms;
                ts2sum2 = max([t2m*t2m*100,ts2sum2])-ts2sum*ts2sum/Nterms;
                ts_out(k)=(csum-2.0*ts1sum*ts2sum/Nterms+ts1sum*ts2sum/Nterms/Nterms)/np.sqrt(ts1sum2*ts2sum2);
            end
            [~,id]=max(ts_out);
            best_shift = id-ishift;
        end
        
        function [best_shift, ts_out]=refine_correlation(ts1,ts2,shift_guess)
            % """Refine a rough guess of shift by evaluating ZNCC for similar values
            %
            % Shifts of *ts1* are tested in the range [-2:2]
            % Refine a rough guess of shift, by trying neighbouring ZNCC values
            % in the range [-2:2]
            %
            % Parameters
            % ----------------
            % ts1 : list_like
            %         The first timeseries
            % ts2 : list_like
            %         The seconds timeseries
            % shift_guess : float
            %         The guess to start from
            %
            % Returns
            % ---------------
            % best_shift : float
            %         The best shift of those tested
            % ts_out : ndarray
            %         Computed correlation values
            % """
            Ns1 = length(ts1);
            Ns2 = length(ts2);
            ts_out = zeros(5,1);

            ishift = int(floor(Ns2/2)) % origin of ts2
            k_offset = shift_guess-2+ishift; % Try shifts starting with this one

            t1m = mean(ts1);
            t2m = mean(ts2);
            
            for k = 1:5
                km = k+k_offset;
                lstart = int(ishift-km);
                if lstart< 1
                    lstart=1;
                end
                lend = int(ishift-km+Ns2);
                imax = int(min([Ns2,Ns1-km+ishift]));
                if lend > imax
                    lend=imax;
                end
                csum = 0;
                ts1sum = 0;
                ts1sum2 = 0;
                ts2sum = 0;
                ts2sum2 = 0;
                Nterms = lend-lstart;
                for l = lstart:lend
                    csum    = csum+ts1(km+l-ishift)*ts2(l);
                    ts1sum  = ts1sum+ts1(km+l-ishift);
                    ts1sum2 = ts1sum2+ts1(km+l-ishift)*ts1(km+l-ishift);
                    ts2sum  = ts2sum+ts2(l);
                    ts2sum2 = ts2sum2+ts2(l)*ts2(l);
                end
                ts1sum2 = max([t1m*t1m*100,ts1sum2])-ts1sum*ts1sum/Nterms;
                ts2sum2 = max([t2m*t2m*100,ts2sum2])-ts2sum*ts2sum/Nterms;
                ts_out(k)=(csum-2.0*ts1sum*ts2sum/Nterms+ts1sum*ts2sum/Nterms/Nterms)/np.sqrt(ts1sum2*ts2sum2);
            end
            [~,id]=max(ts_out);
            best_shift = id+k_offset-ishift;
        end
        
        function ts1_shift = find_shift_pyr(ts1,ts2,nlevels)
            % """
            % Find shift that best aligns two time series
            %
            % The shift that aligns the timeseries ts1 with ts2.
            % This is sought using zero mean normalized cross correlation (ZNCC) in a coarse to fine search with an octave pyramid on nlevels levels.
            %
            % Parameters
            % ----------------
            % ts1 : list_like
            %         The first timeseries
            % ts2 : list_like
            %         The seconds timeseries
            % nlevels : int
            %         Number of levels in pyramid
            %
            % Returns
            % ----------------
            %    ts1_shift : float
            %            How many samples to shift ts1 to align with ts2
            % """
            pyr1 = create_pyramid(ts1,nlevels);
            pyr2 = create_pyramid(ts2,nlevels);
            [ishift, corrfn] = zncc(pyr1{end},pyr2{end});
            for k = nlevels:1
                [ishift, corrfn] = refine_correlation(pyr1{k},pyr2{k},ishift*2);
            end
            ts1_shift = ishift;
        end
        %% the previous part are from crisp. However, it works not so good.
        
        
        %% here starts my implementation of coase-to-fine alignemnt
        function pyr_out = create_gaussian_pyramid(time_series,octaves,sigma)
            pyr_out = {time_series};
            kernel = gaussian_kernel(sigma);
            downsample_factor = 2;
            for i = 1:octaves
                % blur
                ts_blur = np.convolve(pyr_out{end},kernel,'same');
                % numpy needs this, matlab not
%                 if len(pyr_out[-1]) < len(kernel):
%                     id1 = int(np.round((len(kernel) - len(pyr_out[-1])+0.5)*0.5))
%                     ts_blur = ts_blur[id1:id1+len(pyr_out[-1])]
                % downsample
                ts_out = ts_blur(1:downsample_factor:end);
                % append
                pyr_out{end+1}=ts_out;
            end
        end
        
        function toff = coarse_to_fine_corr(x,y,varargin)
            if nargin >= 3
                sigma = varargin{1};
            else
                sigma = 10;
            end
            if nargin >=4
                pyradmids = varargin{2};
            else
                pyradmids = 8;
            end
            
            xpy = cell(pyradmids,1);
            ypy = cell(pyradmids,1);
    
            ind = round(-3*sigma:1:3*sigma);
            % kernel-1d
            kernel = 1/sqrt(2*pi*sigma^2).*exp(-0.5.*ind.^2./sigma^2);
            kernel = kernel./sum(kernel);
            xpy{1} = x;
            ypy{1} = y;
            % gaussian pyramid
            for i = 2:pyradmids
                xd = conv(xpy{i-1},kernel,'same');
                yd = conv(ypy{i-1},kernel,'same');
                xpy{i} = downsample(xd,2);
                ypy{i} = downsample(yd,2);
            end
            % backward xcorr
            n = length(xpy{i});
            [r,d]=xcorr(xpy{i},ypy{i});
            [~,maxid]=max(r);
            lag = d(maxid);
            for i = pyradmids-1:-1:1
%         [r,d]=xcorr(circshift(xpy{i},-lag*2),ypy{i},max(round(n/2),1));
                if lag > 0
                    [r,d]=xcorr(xpy{i}(max(lag*2,1):end),ypy{i}(1:end-lag*2),max(round(n/2),1));
                else
                    try
                        [r,d]=xcorr(xpy{i}(1:end+lag*2),ypy{i}(max(-2*lag,1):end),max(round(n/2),1));
                    catch
                        warning('')
                    end
                end
                [~,maxid]=max(r);
                lag = lag*2 + d(maxid);
                n = round(n/1.8);
            end
            toff = lag;
        end
    end
end

%% this is the corresponding implementation in python using scipy and numpy
% it is different since conv and correlate behaves differently with matlab.


% def coarse_to_fine_corr(ts1, ts2, sigma = 10, nlevels=8):
%     ts1=np.reshape(ts1,(ts1.shape[0]))
%     ts2=np.reshape(ts2,(ts2.shape[0]))
%     # create gaussian pyramid
%     ts1_pyr = create_gaussian_pyramid(ts1,nlevels,sigma)
%     ts2_pyr = create_gaussian_pyramid(ts2,nlevels,sigma)
%     # coarse-to-fine correlation
%     n = int(np.size(ts1_pyr[-1]))
%     # r = signal.correlate(ts1_pyr[-1], ts2_pyr[-1])
%     # shift = np.argmax(r)-len(ts2_pyr[-1]) + 1
%     shifts = call_correlate(ts1_pyr[-1], ts2_pyr[-1], ret_num=1)
%     for k in range(nlevels-1,-1,-1):
%         n = int(max(np.round((n+0.5)*0.5),1))
%         for i, shift in enumerate(shifts):        
%             if shift > 0:
%                 newshift = shift
%                 sig1 = ts1_pyr[k][max(newshift*2-1,0):]
%                 sig2 = ts2_pyr[k][0:(len(ts2_pyr[k])-2*newshift)]
%                 dshift = call_correlate(sig1, sig2, ret_num=1, maxlag=n)
%                 # np.argmax(signal.correlate(sig1, sig2,method='fft'))
%             else:
%                 newshift = shift
%                 sig1 = ts1_pyr[k][0:(len(ts1_pyr[k])+2*newshift+1)]
%                 sig2 = ts2_pyr[k][max(-newshift*2-1,0):]
%                 # r = signal.correlate(sig1, sig2)
%                 # sig1 = ts1_pyr[k]
%                 # sig2 = ts2_pyr[k]
%                 # ind = int(len(r) * 0.5)
%                 # r = r[ind-n:ind+n]
%                 # dshift = np.argmax(r)-len(r) + 1
%                 dshift = call_correlate(sig1, sig2, ret_num=1, maxlag=n)
%             shifts[i] = shifts[i] * 2 + dshift
%     # return ts_out
%     shift = shifts.max()
%     return shift
% 
% def call_correlate(sig1,sig2,ret_num=3, maxlag=None):
%     import scipy.signal as signal
%     r = signal.correlate(sig1, sig2)
%     if not maxlag is None:
%         id1 = int(len(sig2))
%         r = r[id1-1-maxlag:id1+maxlag]
%         dshifts = r.argsort()[-int(ret_num):][::-1]
%         dshifts = dshifts - maxlag
%     else:
%         dshifts = r.argsort()[-int(ret_num):][::-1]
%         dshifts = dshifts - len(sig2) + 1
%     
%     return dshifts