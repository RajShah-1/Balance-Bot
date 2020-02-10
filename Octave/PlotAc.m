function PlotAc(p)
  f = [1, 10, 50, 100, 500, 1000, 5000, 10^4, 5 * 10^4, 10^5, 5 * 10^5, 10^6];
  f = log10(f);
  p = 20*log10(p)
  figure()
  plot(f, p, "-b", "LineWidth", 2);
  xlabel("Frequency in Log scale");
  ylabel("Output voltage in Log scale");
  title("De-Emphasis");
endfunction
