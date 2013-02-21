Grouped-input version of topic_tools/mux.  Switches between sets of inputs.

Example:

```
rosrun multimux multimux --topics topic1 topic2 topic3 --input_prefixes prefix1 prefix2
```

This will subscribe to these topics:

/prefix1/topic1
/prefix1/topic2
/prefix1/topic3
/prefix2/topic1
/prefix2/topic2
/prefix2/topic3

and publish these topics:

/topic1
/topic2
/topic3

multimux supports the same set of service calls that topic_tools/mux
does.  The naming is a bit off, since those service definitions use
the word "topic" to describe which is being selected, but in this case
the *prefix* is what is being selected.

If you run this:

```
rosrun topic_tools mux_select mux /prefix2
```

then data received on the topics:

/prefix2/topic1
/prefix2/topic2
/prefix2/topic3

will be forwarded to the corresponding output topics.
